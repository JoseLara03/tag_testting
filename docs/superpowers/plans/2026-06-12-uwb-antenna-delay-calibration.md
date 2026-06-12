# UWB Antenna Delay Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Calibrate each tag's UWB antenna delay once over BLE NUS against a known reference distance, auto-solve the optimal delay, and persist it in NVS; ranging refuses to start without a valid stored calibration.

**Architecture:** Pure, dependency-free math (`cal_math.c`) is verified by embedded self-test vectors. A persistence/orchestration module (`cal.c`) owns NVS load/store and a request flag. The SS-TWR ranging thread (`uwb_ss_initiator.c`) owns all DW3000 access: it applies the loaded delay, and when a calibration is requested it collects N ranges, solves iteratively, and stores the result. A BLE NUS RX handler parses `cal` commands.

**Tech Stack:** Zephyr RTOS (nCS 3.2.4), nRF52833, Qorvo DW3000 driver, Zephyr NVS, BLE NUS.

**Toolchain note:** Only `arm-zephyr-eabi-gcc` is available (no host gcc/QEMU). Math is verified on-target via `cal selftest` over BLE. Build = `west build`; flash is done by the user, who reports BLE output. Optional: if MinGW gcc is installed, `cal_math.c` + `tests/cal_math/test_cal_math.c` compile and run on host for faster iteration (see Task 6).

---

## File Structure

- **Create `src/cal_math.h`** — POD `struct cal_record`, constants, and pure function declarations. Zero Zephyr deps.
- **Create `src/cal_math.c`** — pure implementations: CRC32, record validate/finalize, solver step, equal split, filtered mean (median/MAD outlier rejection), and `cal_math_selftest()`.
- **Create `src/cal.h` / `src/cal.c`** — NVS persistence, active delay storage, calibration request flag/semaphore, BLE `cal` command parser. Depends on `cal_math` + Zephyr NVS + `ble_log`.
- **Modify `src/ble_log.c` / `src/ble_log.h`** — add `ble_log_set_rx_handler()`; route NUS RX to it.
- **Modify `src/uwb_ss_initiator.c`** — load delay from `cal`, gate ranging on valid calibration, run the iterative calibration procedure on request; refactor one exchange into a reusable helper.
- **Modify `src/main.c`** — call `cal_init()` and register the RX handler before starting ranging.
- **Modify `src/phy_config.h`** — demote `TX_ANT_DLY/RX_ANT_DLY` to documented factory-reference fallback only.
- **Modify `CMakeLists.txt`** — add `src/cal_math.c` and `src/cal.c`.
- **Modify `prj.conf`** — enable `CONFIG_NVS=y` and `CONFIG_FLASH=y` / `CONFIG_FLASH_MAP=y`.
- **Create `tests/cal_math/test_cal_math.c`** — optional host runner (mirrors the self-test vectors).

The `storage_partition` (label "storage", 24 KB @ 0x7a000) already exists in the board DTS — no DTS change needed.

---

## Task 1: Pure math header (`cal_math.h`)

**Files:**
- Create: `src/cal_math.h`

- [ ] **Step 1: Write the header**

```c
#ifndef CAL_MATH_H
#define CAL_MATH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Stored-record identity / layout guards. */
#define CAL_MAGIC               0xCA11B000u
#define CAL_VERSION             1u

/* Solver slope: combined (TX+RX) antenna-delay unit -> reported distance.
 * Increasing the combined delay by 1 unit decreases reported distance by
 * c * 0.5 * DWT_TIME_UNITS = ~2.34 mm. Stored x1000 for integer math. */
#define CAL_MM_PER_UNIT_X1000   2340

/* Clamp for the combined (TX+RX) antenna delay. */
#define CAL_MAX_TOTAL_DLY       65000u

/* Maximum ranging samples processed per calibration iteration. */
#define CAL_MAX_SAMPLES         128u

/* Persisted calibration record. Explicit padding keeps the layout (and thus
 * the CRC) stable across compilers. CRC is computed over every byte preceding
 * the crc32 field. */
struct cal_record {
    uint32_t magic;       /* CAL_MAGIC */
    uint8_t  version;     /* CAL_VERSION */
    uint8_t  phy_option;  /* CONFIG_OPTION value the cal was taken under */
    uint16_t tx_ant_dly;
    uint16_t rx_ant_dly;
    uint16_t _pad;        /* explicit padding for stable layout */
    uint32_t ref_mm;      /* reference distance used (traceability) */
    uint16_t residual_mm; /* achieved residual error */
    uint16_t _pad2;       /* explicit padding for stable layout */
    uint32_t crc32;       /* integrity, computed last */
};

/* Standard CRC-32 (poly 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF). */
uint32_t cal_crc32(const void *data, size_t len);

/* Populate magic/version and compute crc32 over the rest of the record. */
void cal_record_finalize(struct cal_record *r);

/* True if magic, version, phy_option and crc32 all check out. */
bool cal_record_valid(const struct cal_record *r, uint8_t expected_phy);

/* Given the mean measured distance and the reference (both mm) and the current
 * combined delay, return the new combined delay (clamped to [0, MAX]). */
uint16_t cal_solve_step(int32_t measured_mm, int32_t ref_mm, uint16_t cur_total_dly);

/* Split a combined delay equally into TX/RX (rx gets the odd remainder). */
void cal_split_dly(uint16_t total, uint16_t *tx, uint16_t *rx);

/* Mean of samples after median/MAD outlier rejection. Returns false if n==0
 * or all samples are rejected. out_kept = number of inliers used. */
bool cal_filtered_mean(const int32_t *samples, size_t n,
                       int32_t *out_mean, size_t *out_kept);

/* Run built-in assertion vectors. Returns the number of failed checks
 * (0 == all pass). */
int cal_math_selftest(void);

#endif /* CAL_MATH_H */
```

- [ ] **Step 2: Commit**

```bash
git add src/cal_math.h
git commit -m "feat(cal): pure calibration-math interface and record layout"
```

---

## Task 2: CRC32 + record validation (`cal_math.c` part 1)

**Files:**
- Create: `src/cal_math.c`

- [ ] **Step 1: Create `cal_math.c` with CRC32, finalize, validate, and a stub self-test**

```c
#include "cal_math.h"
#include <string.h>

/* ---- CRC-32 (IEEE 802.3, reflected) ---------------------------------------- */
uint32_t cal_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

/* Bytes covered by the CRC: everything before the crc32 field. */
#define CAL_CRC_LEN  (offsetof(struct cal_record, crc32))

void cal_record_finalize(struct cal_record *r)
{
    r->magic   = CAL_MAGIC;
    r->version = CAL_VERSION;
    r->_pad    = 0;
    r->_pad2   = 0;
    r->crc32   = cal_crc32(r, CAL_CRC_LEN);
}

bool cal_record_valid(const struct cal_record *r, uint8_t expected_phy)
{
    if (r->magic != CAL_MAGIC) {
        return false;
    }
    if (r->version != CAL_VERSION) {
        return false;
    }
    if (r->phy_option != expected_phy) {
        return false;
    }
    return r->crc32 == cal_crc32(r, CAL_CRC_LEN);
}

int cal_math_selftest(void)
{
    int fails = 0;

    /* CRC32 known-answer: "123456789" -> 0xCBF43926. */
    if (cal_crc32("123456789", 9) != 0xCBF43926u) {
        fails++;
    }

    /* Record round-trip: finalize then validate must pass for matching phy
     * and fail for a different phy. */
    struct cal_record r = {0};
    r.phy_option = 7;
    r.tx_ant_dly = 16371;
    r.rx_ant_dly = 16371;
    r.ref_mm = 2000;
    cal_record_finalize(&r);
    if (!cal_record_valid(&r, 7)) {
        fails++;
    }
    if (cal_record_valid(&r, 9)) {
        fails++;
    }
    /* Corrupt a byte -> CRC must reject. */
    r.tx_ant_dly ^= 0x01;
    if (cal_record_valid(&r, 7)) {
        fails++;
    }

    return fails;
}
```

- [ ] **Step 2: Add `src/cal_math.c` to the build**

In `CMakeLists.txt`, inside the `target_sources(app PRIVATE ...)` list (after `src/phy_config.c`), add:

```cmake
    src/cal_math.c
```

- [ ] **Step 3: Build to verify it compiles**

Run: `west build` (from the project root, existing build config).
Expected: build succeeds, no warnings on `cal_math.c`.

- [ ] **Step 4: Commit**

```bash
git add src/cal_math.c CMakeLists.txt
git commit -m "feat(cal): CRC32 and record validation with self-test vectors"
```

---

## Task 3: Solver step + equal split (`cal_math.c` part 2)

**Files:**
- Modify: `src/cal_math.c`

- [ ] **Step 1: Add the solver and split functions**

Insert before `cal_math_selftest`:

```c
/* Round-to-nearest signed integer division (den must be > 0). */
static int32_t div_round_pos(int32_t num, int32_t den)
{
    if (num >= 0) {
        return (num + den / 2) / den;
    }
    return -(((-num) + den / 2) / den);
}

uint16_t cal_solve_step(int32_t measured_mm, int32_t ref_mm, uint16_t cur_total_dly)
{
    /* err > 0 => measuring too far => increase delay to pull distance down. */
    int32_t err_mm = measured_mm - ref_mm;
    int32_t delta_units = div_round_pos(err_mm * 1000, CAL_MM_PER_UNIT_X1000);

    int32_t new_total = (int32_t)cur_total_dly + delta_units;

    if (new_total < 0) {
        new_total = 0;
    }
    if (new_total > (int32_t)CAL_MAX_TOTAL_DLY) {
        new_total = (int32_t)CAL_MAX_TOTAL_DLY;
    }
    return (uint16_t)new_total;
}

void cal_split_dly(uint16_t total, uint16_t *tx, uint16_t *rx)
{
    *tx = total / 2u;
    *rx = total - *tx;
}
```

- [ ] **Step 2: Extend `cal_math_selftest` with solver/split vectors**

Inside `cal_math_selftest`, before `return fails;`, add:

```c
    /* Solver: 234 mm too far / 2.34 mm-per-unit = +100 units. */
    if (cal_solve_step(2234, 2000, 32742) != 32842) {
        fails++;
    }
    /* Solver: 234 mm too short = -100 units. */
    if (cal_solve_step(1766, 2000, 32742) != 32642) {
        fails++;
    }
    /* Solver clamps at zero (cannot go negative). */
    if (cal_solve_step(0, 100000, 10) != 0) {
        fails++;
    }
    /* Equal split: even and odd totals. */
    uint16_t tx, rx;
    cal_split_dly(32742, &tx, &rx);
    if (tx != 16371 || rx != 16371) {
        fails++;
    }
    cal_split_dly(33, &tx, &rx);
    if (tx != 16 || rx != 17) {
        fails++;
    }
```

- [ ] **Step 3: Build**

Run: `west build`
Expected: compiles cleanly.

- [ ] **Step 4: Commit**

```bash
git add src/cal_math.c
git commit -m "feat(cal): linear solver step and equal TX/RX split"
```

---

## Task 4: Filtered mean with outlier rejection (`cal_math.c` part 3)

**Files:**
- Modify: `src/cal_math.c`

- [ ] **Step 1: Add the filtered-mean implementation**

Insert before `cal_math_selftest`:

```c
static void sort_i32(int32_t *a, size_t n)
{
    /* Insertion sort: n <= CAL_MAX_SAMPLES (128), simple and allocation-free. */
    for (size_t i = 1; i < n; i++) {
        int32_t key = a[i];
        size_t j = i;
        while (j > 0 && a[j - 1] > key) {
            a[j] = a[j - 1];
            j--;
        }
        a[j] = key;
    }
}

static int32_t median_sorted(const int32_t *a, size_t n)
{
    return a[n / 2];  /* upper-middle for even n; adequate for outlier centring */
}

bool cal_filtered_mean(const int32_t *samples, size_t n,
                       int32_t *out_mean, size_t *out_kept)
{
    if (n == 0 || n > CAL_MAX_SAMPLES) {
        return false;
    }

    int32_t work[CAL_MAX_SAMPLES];
    memcpy(work, samples, n * sizeof(int32_t));
    sort_i32(work, n);
    int32_t med = median_sorted(work, n);

    /* MAD = median of absolute deviations from the median. */
    int32_t devs[CAL_MAX_SAMPLES];
    for (size_t i = 0; i < n; i++) {
        int32_t d = work[i] - med;
        devs[i] = (d < 0) ? -d : d;
    }
    sort_i32(devs, n);
    int32_t mad = median_sorted(devs, n);
    if (mad < 1) {
        mad = 1;  /* floor: avoid rejecting everything when samples are tight */
    }

    int64_t sum = 0;
    size_t kept = 0;
    int32_t limit = 6 * mad;
    for (size_t i = 0; i < n; i++) {
        int32_t d = samples[i] - med;
        if (d < 0) {
            d = -d;
        }
        if (d <= limit) {
            sum += samples[i];
            kept++;
        }
    }
    if (kept == 0) {
        return false;
    }
    *out_mean = (int32_t)(sum / (int64_t)kept);
    *out_kept = kept;
    return true;
}
```

- [ ] **Step 2: Extend `cal_math_selftest` with a filtered-mean vector**

Inside `cal_math_selftest`, before `return fails;`, add:

```c
    /* Filtered mean: one gross outlier (5000) must be rejected. */
    int32_t s[] = {100, 102, 98, 101, 99, 5000};
    int32_t mean;
    size_t kept;
    if (!cal_filtered_mean(s, 6, &mean, &kept)) {
        fails++;
    } else {
        if (kept != 5) {
            fails++;
        }
        if (mean < 98 || mean > 102) {
            fails++;
        }
    }
```

- [ ] **Step 3: Build**

Run: `west build`
Expected: compiles cleanly.

- [ ] **Step 4: Commit**

```bash
git add src/cal_math.c
git commit -m "feat(cal): median/MAD filtered mean for sample rejection"
```

---

## Task 5: Enable NVS in configuration

**Files:**
- Modify: `prj.conf`

- [ ] **Step 1: Append NVS config**

Add to the end of `prj.conf`:

```
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_NVS=y
```

- [ ] **Step 2: Build**

Run: `west build`
Expected: compiles and links (NVS subsystem pulled in).

- [ ] **Step 3: Commit**

```bash
git add prj.conf
git commit -m "build(cal): enable NVS, flash map for calibration storage"
```

---

## Task 6 (OPTIONAL): Host test runner

Only do this task if a host `gcc` is available (e.g., MinGW-w64). It does not affect firmware. Skip if no host compiler — the on-target `cal selftest` (Task 9) is the authoritative check.

**Files:**
- Create: `tests/cal_math/test_cal_math.c`

- [ ] **Step 1: Write the host runner**

```c
#include "../../src/cal_math.h"
#include <stdio.h>

int main(void)
{
    int fails = cal_math_selftest();
    printf("cal_math_selftest: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Compile and run (only if gcc present)**

Run: `gcc -I src tests/cal_math/test_cal_math.c src/cal_math.c -o build/test_cal_math && ./build/test_cal_math`
Expected: `cal_math_selftest: 0 failure(s)` and exit code 0.

- [ ] **Step 3: Commit**

```bash
git add tests/cal_math/test_cal_math.c
git commit -m "test(cal): host runner for cal_math self-test vectors"
```

---

## Task 7: NUS RX handler hook (`ble_log`)

**Files:**
- Modify: `src/ble_log.h`
- Modify: `src/ble_log.c:42-45`

- [ ] **Step 1: Declare the handler type and setter in `ble_log.h`**

Replace the body of `ble_log.h` (between the include guard) with:

```c
#ifndef BLE_LOG_H
#define BLE_LOG_H

#include <stdint.h>

typedef void (*ble_rx_handler_t)(const uint8_t *data, uint16_t len);

int  ble_log_init(void);
void ble_log_wait_ready(void);
void ble_log_send(const char *msg);

/* Register a handler invoked from the NUS RX callback (nrfxlib BT RX thread)
 * for every write the central sends. Pass NULL to unregister. */
void ble_log_set_rx_handler(ble_rx_handler_t handler);

#endif /* BLE_LOG_H */
```

- [ ] **Step 2: Route RX to the handler in `ble_log.c`**

Replace the existing `nus_received` stub (lines 42-45):

```c
static void nus_received(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    /* TX-only — ignore incoming data */
}
```

with:

```c
static ble_rx_handler_t rx_handler;

void ble_log_set_rx_handler(ble_rx_handler_t handler)
{
    rx_handler = handler;
}

static void nus_received(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(conn);
    if (rx_handler) {
        rx_handler(data, len);
    }
}
```

- [ ] **Step 3: Build**

Run: `west build`
Expected: compiles cleanly.

- [ ] **Step 4: Commit**

```bash
git add src/ble_log.h src/ble_log.c
git commit -m "feat(ble): NUS RX handler hook for command input"
```

---

## Task 8: NVS persistence + command parser (`cal.c` / `cal.h`)

**Files:**
- Create: `src/cal.h`
- Create: `src/cal.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write `cal.h`**

```c
#ifndef CAL_H
#define CAL_H

#include <stdint.h>
#include <stdbool.h>

/* Mount NVS, load the stored calibration for the current PHY, and register the
 * BLE `cal` command handler. Returns true if a valid record was loaded. */
bool cal_init(void);

/* Active antenna delays. Meaningful only when cal_is_valid() is true. */
void cal_get_ant_dly(uint16_t *tx, uint16_t *rx);

/* True if a valid calibration is currently loaded/active. */
bool cal_is_valid(void);

/* Erase the stored record and mark calibration invalid. 0 on success. */
int cal_clear(void);

/* ---- ranging-thread side ---- */

/* If a `cal <mm>` command was queued, copy the reference into out_ref_mm,
 * clear the request, and return true. */
bool cal_take_request(uint32_t *out_ref_mm);

/* Block until a calibration request arrives (used while ranging is gated). */
void cal_wait_request(void);

/* Persist a solved result and activate it. 0 on success. */
int cal_store(uint16_t tx, uint16_t rx, uint32_t ref_mm, uint16_t residual_mm);

#endif /* CAL_H */
```

- [ ] **Step 2: Write `cal.c`**

```c
#include "cal.h"
#include "cal_math.h"
#include "ble_log.h"
#include "phy_config.h"   /* CONFIG_OPTION */

#include <zephyr/kernel.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <string.h>
#include <stdlib.h>

#define CAL_NVS_PARTITION   storage_partition
#define CAL_NVS_ID          1

static struct nvs_fs fs;
static bool          fs_ready;

static struct cal_record active;
static bool             active_valid;

static K_SEM_DEFINE(cal_req_sem, 0, 1);
static volatile uint32_t cal_req_ref_mm;
static volatile bool     cal_req_pending;

/* ---- NVS bring-up ---------------------------------------------------------- */
static int nvs_bringup(void)
{
    const struct flash_area *fa;
    int rc = flash_area_open(FIXED_PARTITION_ID(CAL_NVS_PARTITION), &fa);
    if (rc) {
        return rc;
    }

    struct flash_pages_info info;
    rc = flash_get_page_info_by_offs(flash_area_get_device(fa),
                                     fa->fa_off, &info);
    if (rc) {
        flash_area_close(fa);
        return rc;
    }

    fs.flash_device = flash_area_get_device(fa);
    fs.offset       = fa->fa_off;
    fs.sector_size  = info.size;
    fs.sector_count = (uint16_t)(fa->fa_size / info.size);
    flash_area_close(fa);

    rc = nvs_mount(&fs);
    if (rc == 0) {
        fs_ready = true;
    }
    return rc;
}

/* ---- command parser (runs in BT RX thread) -------------------------------- */
static void cal_on_rx(const uint8_t *data, uint16_t len)
{
    char buf[24];
    uint16_t n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);

    memcpy(buf, data, n);
    buf[n] = '\0';
    /* strip trailing CR/LF */
    while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
        buf[--n] = '\0';
    }

    if (strcmp(buf, "cal clear") == 0) {
        ble_log_send(cal_clear() == 0 ? "CAL cleared\n" : "CAL FAIL nvs\n");
        return;
    }
    if (strcmp(buf, "cal status") == 0) {
        if (active_valid) {
            char msg[20];
            (void)snprintf(msg, sizeof(msg), "CAL %u/%u\n",
                           active.tx_ant_dly, active.rx_ant_dly);
            ble_log_send(msg);
        } else {
            ble_log_send("CAL REQUIRED\n");
        }
        return;
    }
    if (strcmp(buf, "cal selftest") == 0) {
        char msg[20];
        (void)snprintf(msg, sizeof(msg), "SELFTEST %d\n", cal_math_selftest());
        ble_log_send(msg);
        return;
    }
    if (strncmp(buf, "cal ", 4) == 0) {
        char *end;
        long mm = strtol(buf + 4, &end, 10);
        if (end != buf + 4 && mm > 0) {
            cal_req_ref_mm = (uint32_t)mm;
            cal_req_pending = true;
            k_sem_give(&cal_req_sem);
            ble_log_send("CAL start\n");
            return;
        }
    }
    ble_log_send("CAL ERR usage: cal <mm>\n");
}

/* ---- public API ------------------------------------------------------------ */
bool cal_init(void)
{
    ble_log_set_rx_handler(cal_on_rx);

    if (nvs_bringup() != 0) {
        active_valid = false;
        return false;
    }

    struct cal_record r;
    ssize_t got = nvs_read(&fs, CAL_NVS_ID, &r, sizeof(r));
    if (got == sizeof(r) && cal_record_valid(&r, (uint8_t)CONFIG_OPTION)) {
        active = r;
        active_valid = true;
        return true;
    }
    active_valid = false;
    return false;
}

void cal_get_ant_dly(uint16_t *tx, uint16_t *rx)
{
    *tx = active.tx_ant_dly;
    *rx = active.rx_ant_dly;
}

bool cal_is_valid(void)
{
    return active_valid;
}

int cal_clear(void)
{
    if (!fs_ready) {
        return -1;
    }
    int rc = nvs_delete(&fs, CAL_NVS_ID);
    if (rc == 0) {
        active_valid = false;
    }
    return rc;
}

bool cal_take_request(uint32_t *out_ref_mm)
{
    if (!cal_req_pending) {
        return false;
    }
    *out_ref_mm = cal_req_ref_mm;
    cal_req_pending = false;
    return true;
}

void cal_wait_request(void)
{
    k_sem_take(&cal_req_sem, K_FOREVER);
}

int cal_store(uint16_t tx, uint16_t rx, uint32_t ref_mm, uint16_t residual_mm)
{
    if (!fs_ready) {
        return -1;
    }

    struct cal_record r = {0};
    r.phy_option  = (uint8_t)CONFIG_OPTION;
    r.tx_ant_dly  = tx;
    r.rx_ant_dly  = rx;
    r.ref_mm      = ref_mm;
    r.residual_mm = residual_mm;
    cal_record_finalize(&r);

    ssize_t rc = nvs_write(&fs, CAL_NVS_ID, &r, sizeof(r));
    if (rc < 0) {
        return (int)rc;
    }
    active = r;
    active_valid = true;
    return 0;
}
```

- [ ] **Step 3: Add `src/cal.c` to the build**

In `CMakeLists.txt`, in the `target_sources` list after `src/cal_math.c`, add:

```cmake
    src/cal.c
```

- [ ] **Step 4: Build**

Run: `west build`
Expected: compiles and links. If the linker reports an undefined `snprintf`, confirm `CONFIG_NEWLIB` is not required — `snprintf` with `%u/%d` and no floats is provided by picolibc; no float printf is used.

- [ ] **Step 5: Commit**

```bash
git add src/cal.h src/cal.c CMakeLists.txt
git commit -m "feat(cal): NVS persistence and BLE cal command parser"
```

---

## Task 9: Integrate calibration into the ranging thread

**Files:**
- Modify: `src/uwb_ss_initiator.c`

- [ ] **Step 1: Add includes and helper constants**

After the existing `#include "phy_config.h"` (line 18), add:

```c
#include "cal.h"
#include "cal_math.h"
```

After the `RNG_SLOW_MS` define (line 34), add:

```c
/* Calibration procedure parameters. */
#define CAL_SAMPLES_PER_ITER  100U   /* ranges averaged per iteration */
#define CAL_MAX_ITERS         4U     /* give up after this many corrections */
#define CAL_ACCEPT_MM         15     /* residual error considered converged */
```

- [ ] **Step 2: Refactor one ranging exchange into a reusable helper**

Replace the body of the `if (evt == EVT_RXFCG)` distance computation in `ss_twr_fn` by extracting it. Add this helper function immediately before `ss_twr_fn` (after the `INT_RX_PHASE` define, line 159):

```c
/*
 * Run a single SS-TWR exchange. On a valid response, writes the measured
 * distance in millimetres to *out_mm and returns true. Returns false on
 * timeout, RX error, or an unexpected frame. Assumes interrupts/antenna delay
 * are already configured by the caller.
 */
static bool do_one_range(int32_t *out_mm)
{
    dwt_setinterrupt(INT_RX_PHASE, 0, DWT_ENABLE_INT_ONLY);

    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    frame_seq_nb++;

    irq_evt_t evt = wait_event(K_MSEC(20));

    if (evt != EVT_RXFCG) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return false;
    }

    uint16_t flen = dwt_getframelength();
    if (flen <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buf, flen, 0);
    }
    rx_buf[ALL_MSG_SN_IDX] = 0;

    if (memcmp(rx_buf, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        return false;
    }

    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
    double clock_offset_ratio =
        ((double)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    uint32_t poll_rx_ts = get_ts_4b(&rx_buf[RESP_MSG_POLL_RX_TS_IDX]);
    uint32_t resp_tx_ts = get_ts_4b(&rx_buf[RESP_MSG_RESP_TX_TS_IDX]);

    int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

    double tof = ((rtd_init - rtd_resp * (1 - clock_offset_ratio)) / 2.0)
                 * DWT_TIME_UNITS;
    *out_mm = (int32_t)(tof * SPEED_OF_LIGHT * 1000.0);
    return true;
}
```

- [ ] **Step 3: Add the calibration procedure**

Add immediately after `do_one_range`:

```c
/* Apply a combined antenna delay to the DW3000 (split equally TX/RX). */
static void apply_total_dly(uint16_t total, uint16_t *tx, uint16_t *rx)
{
    cal_split_dly(total, tx, rx);
    dwt_settxantennadelay(*tx);
    dwt_setrxantennadelay(*rx);
}

/*
 * Iterative auto-solve: collect CAL_SAMPLES_PER_ITER ranges, reject outliers,
 * correct the combined antenna delay toward ref_mm, repeat until the residual
 * is within CAL_ACCEPT_MM or CAL_MAX_ITERS is exhausted. Stores to NVS on
 * success. Reports progress over BLE.
 */
static void run_calibration(uint32_t ref_mm)
{
    static int32_t samples[CAL_MAX_SAMPLES];

    uint16_t tx, rx;
    /* Seed from the active value if valid, else the factory reference. */
    uint16_t total = active_total_seed();
    apply_total_dly(total, &tx, &rx);

    for (uint32_t it = 0; it < CAL_MAX_ITERS; it++) {
        size_t got = 0;
        for (uint32_t i = 0; i < CAL_SAMPLES_PER_ITER; i++) {
            int32_t mm;
            if (do_one_range(&mm)) {
                samples[got++] = mm;
            }
            k_sleep(K_MSEC(5));
        }

        int32_t mean;
        size_t kept;
        if (got < CAL_SAMPLES_PER_ITER / 4 ||
            !cal_filtered_mean(samples, got, &mean, &kept)) {
            twr_log("CAL FAIL no-resp\n");
            return;
        }

        int32_t err = mean - (int32_t)ref_mm;
        int32_t abserr = (err < 0) ? -err : err;
        twr_log("CAL it%u e=%dmm\n", it + 1, err);

        if (abserr <= CAL_ACCEPT_MM) {
            if (cal_store(tx, rx, ref_mm, (uint16_t)abserr) == 0) {
                twr_log("CAL OK %u/%u\n", tx, rx);
            } else {
                twr_log("CAL FAIL nvs\n");
            }
            return;
        }

        total = cal_solve_step(mean, (int32_t)ref_mm, total);
        apply_total_dly(total, &tx, &rx);
    }
    twr_log("CAL FAIL res\n");
}
```

Add this small seed helper before `run_calibration`:

```c
/* Combined antenna-delay seed for a fresh calibration run. */
static uint16_t active_total_seed(void)
{
    if (cal_is_valid()) {
        uint16_t tx, rx;
        cal_get_ant_dly(&tx, &rx);
        return (uint16_t)(tx + rx);
    }
    return (uint16_t)(TX_ANT_DLY + RX_ANT_DLY);  /* factory reference fallback */
}
```

- [ ] **Step 4: Rewrite `ss_twr_fn` to gate ranging and honour cal requests**

Replace the entire `ss_twr_fn` function (lines 161-238) with:

```c
static void ss_twr_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    dwt_setcallbacks(cb_txdone, cb_rxok, cb_rxto, cb_rxerr, NULL, NULL, NULL);
    port_set_dwic_isr(dwt_isr);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    bool ranging = cal_is_valid();
    if (ranging) {
        uint16_t tx, rx;
        cal_get_ant_dly(&tx, &rx);
        dwt_settxantennadelay(tx);
        dwt_setrxantennadelay(rx);
        twr_log("SS-TWR start\n");
    } else {
        twr_log("CAL REQUIRED\n");
    }

    while (1) {
        uint32_t ref_mm;
        if (cal_take_request(&ref_mm)) {
            run_calibration(ref_mm);
            ranging = cal_is_valid();
            if (ranging) {
                uint16_t tx, rx;
                cal_get_ant_dly(&tx, &rx);
                dwt_settxantennadelay(tx);
                dwt_setrxantennadelay(rx);
            }
        }

        if (!ranging) {
            cal_wait_request();   /* block until a cal command arrives */
            continue;
        }

        int32_t mm;
        if (do_one_range(&mm)) {
            int32_t v = mm;
            const char *sign = (v < 0) ? "-" : "";
            if (v < 0) {
                v = -v;
            }
            twr_log("D:%s%d.%02dm\n", sign, v / 1000, (v % 1000) / 10);
        }

        uint32_t wait_ms = ss_moving ? RNG_FAST_MS : RNG_SLOW_MS;
        k_sem_take(&range_tick, K_MSEC(wait_ms));
    }
}
```

- [ ] **Step 5: Build**

Run: `west build`
Expected: compiles and links. Watch for unused-variable warnings on the old timing macros (`POLL_TX_TO_RESP_RX_DLY_UUS` etc. are still used).

- [ ] **Step 6: Commit**

```bash
git add src/uwb_ss_initiator.c
git commit -m "feat(cal): gate ranging on stored calibration; iterative auto-solve over BLE"
```

---

## Task 10: Wire `cal_init` into startup + demote phy_config defines

**Files:**
- Modify: `src/main.c:36`
- Modify: `src/phy_config.h:8-14`

- [ ] **Step 1: Call `cal_init()` before starting ranging in `main.c`**

Add the include near the top of `main.c` (after `#include "uwb_ss_initiator.h"`):

```c
#include "cal.h"
```

Replace, inside the `if (uwb_init(3) == 0)` block, this line:

```c
        uwb_ss_initiator_start();
```

with:

```c
        if (cal_init()) {
            ble_log_send("CAL loaded\n");
        } else {
            ble_log_send("CAL REQUIRED\n");
        }
        uwb_ss_initiator_start();
```

- [ ] **Step 2: Demote the antenna-delay defines in `phy_config.h`**

Replace the comment block and defines at `phy_config.h:8-14`:

```c
/* Calibrated UWB antenna delay (device time units), split equally between TX
 * and RX. Determined by SS-TWR ranging against a DWM3001CDK anchor at a known
 * reference distance (see src/uwb_ss_initiator.c). The sum (TX+RX) is the
 * device antenna delay; increasing it decreases the reported distance.
 * TODO: persist this in nRF52 NVS instead of hard-coding (out of scope now). */
#define TX_ANT_DLY 16371
#define RX_ANT_DLY 16371
```

with:

```c
/* Factory-reference antenna delay (device time units), split equally between
 * TX and RX. This is ONLY the seed/fallback used by the calibration routine on
 * an uncalibrated unit — the active value is now per-unit and stored in NVS by
 * src/cal.c (see the `cal <mm>` BLE command). Each unit must be calibrated
 * before ranging starts (CONFIG_OPTION-specific). */
#define TX_ANT_DLY 16371
#define RX_ANT_DLY 16371
```

- [ ] **Step 3: Build**

Run: `west build`
Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add src/main.c src/phy_config.h
git commit -m "feat(cal): load calibration at startup; phy_config delays are now fallback-only"
```

---

## Task 11: On-target verification

**Files:** none (hardware checkpoint)

- [ ] **Step 1: Flash and verify self-test (math correctness)**

Flash the build. Connect a BLE NUS central. Send `cal selftest`.
Expected: `SELFTEST 0` (zero failures). Any non-zero value means a math vector failed — stop and fix `cal_math.c`.

- [ ] **Step 2: Verify the gating (NVS obligatorio)**

On a unit with no stored calibration (or after `cal clear`), power-cycle.
Expected: `CAL REQUIRED` over NUS and NO `D:x.xxm` ranging output.

- [ ] **Step 3: Run a calibration**

Place the tag at a known reference distance from the anchor (e.g., 2.000 m). Send `cal 2000`.
Expected: `CAL start`, then per-iteration `CAL itN e=±NNmm` lines, ending in `CAL OK <tx>/<rx>`. Ranging `D:` output then begins and reads within ±15 mm of the reference.

- [ ] **Step 4: Verify persistence**

Power-cycle the tag.
Expected: `CAL loaded` at boot, ranging starts immediately, distance still within tolerance — confirms NVS round-trip.

- [ ] **Step 5: Verify clear**

Send `cal clear`, then power-cycle.
Expected: `CAL cleared`, then `CAL REQUIRED` at boot with ranging gated again.

- [ ] **Step 6: Final commit (docs)**

Update `CLAUDE.md` "UWB antenna delay (calibration)" key-pattern note to reflect that the value is now solved on-device and persisted in NVS via the `cal <mm>` command (removing the "Planned" caveat). Then:

```bash
git add CLAUDE.md
git commit -m "docs(cal): document on-device NVS calibration workflow"
```

---

## Self-Review

**Spec coverage:**
- Auto-solve via BLE with reference distance → Tasks 8 (`cal <mm>`), 9 (`run_calibration`). ✓
- Linear iterative correction, ~2.34 mm/unit, max 4 iters, N=100, 15 mm threshold → Task 3 (`cal_solve_step`), Task 9 (loop params). ✓
- NVS mandatory; ranging blocked without valid record; `CAL REQUIRED` → Task 9 (`ranging` gate, `cal_wait_request`), Task 10 (boot message). ✓
- Versioned record with magic/version/phy_option/crc32 guards → Tasks 1, 2. ✓
- Equal TX/RX split → Task 3 (`cal_split_dly`), Task 9 (`apply_total_dly`). ✓
- Outlier rejection (median ± k·MAD) → Task 4. ✓
- Commands `cal <mm>`, `cal clear`, `cal status` (+ `cal selftest`) → Task 8. ✓
- NUS RX channel (was TX-only) → Task 7. ✓
- Error reporting: no-resp / no-converge / nvs / malformed → Task 8 parser + Task 9 procedure. ✓
- phy_config demoted to fallback → Task 10. ✓
- Tests for solver math, record validation, CRC32, outlier rejection → Tasks 2-4 self-test vectors + Task 11 on-target `cal selftest`; optional host runner Task 6. ✓

**Placeholder scan:** No TBD/TODO left in plan steps; every code step shows full code. ✓

**Type consistency:** `struct cal_record` fields used identically across `cal_math.c`, `cal.c`. `cal_solve_step`/`cal_split_dly`/`cal_filtered_mean`/`cal_record_valid` signatures match header. `apply_total_dly`/`active_total_seed`/`do_one_range`/`run_calibration` are defined before use within `uwb_ss_initiator.c`. `ble_rx_handler_t` matches `cal_on_rx` signature `(const uint8_t*, uint16_t)`. ✓

**Scope check:** Single subsystem (per-unit calibration). Focused enough for one plan. ✓
