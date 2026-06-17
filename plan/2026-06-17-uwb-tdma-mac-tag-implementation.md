# UWB TDMA MAC — Tag-Side Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the tag a TDMA citizen — it parses the gateway beacon, joins the PAN to get a leased slot, discovers its best anchors, and ranges only inside its slot, instead of free-running.

**Architecture:** A portable, host-testable MAC core (`uwb_net` state machine + extensions to the `uwb_frame_802_15_4z` frame module) sits above a thin target-only `radio_ops` seam that wraps the DW3000. The state machine is a pure Mealy function `uwb_net_handle(ctx, event) -> action-flags`; a target runner turns radio results into events and action-flags into `dwt_*` calls, reusing today's `do_one_range_anchor()` / `pos_solve()` / BLE-sender.

**Tech Stack:** C99, Zephyr RTOS (nCS 3.2.4), Nordic nRF52833 + Qorvo DW3000, CMSIS-DSP. Host unit tests built with WinLibs GCC.

**Specs:** [`spec/2026-06-17-uwb-mac-protocol-contract.md`](../spec/2026-06-17-uwb-mac-protocol-contract.md), [`spec/2026-06-17-tag-network-layer-design.md`](../spec/2026-06-17-tag-network-layer-design.md).

## Global Constraints

- **Portable units have ZERO hardware deps.** `uwb_frame_802_15_4z.*` and `uwb_net.*` may include only `<stdint.h> <stddef.h> <stdbool.h> <string.h> <errno.h>` — never `<zephyr/...>`, `deca_*`, GPIO, or `k_*`. (This is what makes them host-testable.)
- **Host test compiler (exact, not on PATH):** `GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"` — use WinLibs gcc, **not** clang. Flags: `-std=c99 -Wall -Wextra -Werror`.
- **Addresses are 16-bit little-endian short.** Gateway `0x0000`, unassociated tag `0xFFFE`, broadcast `0xFFFF`.
- **New function codes:** BEACON `0xE5`, JOIN_REQ `0xE6`, GRANT `0xE7`, KEEPALIVE `0xE8`, RELEASE `0xE9`. (DISC `0xE2`, MPOL `0xE3`, RESP `0xE4` already exist; `0xE0/0xE1` reserved for bench cal.)
- **Protocol version `UWB_PROTO_VER = 1`.** `N_CFP = 12`, `N_CAP = 4`.
- **Lifecycle defaults:** `lease_superframes = 50`, beacon-miss limit `M = 3`, join-retry limit `N = 4`, keepalive renews at 50 % of lease.
- **Tier cadence (superframes between slots):** FAST `1`, SLOW `5`, IDLE `25`.
- **CMakeLists:** new `src/*.c` must be added to `target_sources(app PRIVATE ...)`.
- **Commit on the `feat/uwb-tdma-mac` branch.** Do not merge to master.

---

### Task 0: Bring the 4z frame module onto the working branch

The frame module the whole plan extends lives only on `feat/uwb-frame-802-15-4z`. Merge it in first.

**Files:**
- Adds (via merge): `src/uwb_frame_802_15_4z.c`, `src/uwb_frame_802_15_4z.h`, `tests/uwb_frame/test_uwb_frame.c`

- [ ] **Step 1: Merge the frame branch**

```bash
git checkout feat/uwb-tdma-mac
git merge --no-ff feat/uwb-frame-802-15-4z -m "merge(uwb): 802.15.4z frame module into TDMA MAC branch"
```

- [ ] **Step 2: Verify files are present**

Run: `ls src/uwb_frame_802_15_4z.* tests/uwb_frame/test_uwb_frame.c`
Expected: all three paths listed, no error.

- [ ] **Step 3: Add the frame module to the firmware build**

In `CMakeLists.txt`, inside `target_sources(app PRIVATE ...)`, add the line `    src/uwb_frame_802_15_4z.c` (next to the other `src/*.c`).

- [ ] **Step 4: Establish the host-test baseline**

Run:
```bash
GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/uwb_frame/test_uwb_frame.c src/uwb_frame_802_15_4z.c -o /tmp/t_frame.exe && /tmp/t_frame.exe; echo "exit=$?"
```
Expected: existing tests print no `FAIL` lines and `exit=0`.

- [ ] **Step 5: Commit the CMake change**

```bash
git add CMakeLists.txt
git commit -m "build(uwb): compile uwb_frame_802_15_4z into firmware"
```

---

### Task 1: Frame module — new constants + a u32 reader

Add the contract's new types/addresses/offsets and a little-endian 32-bit reader (the beacon frame counter needs it; the module currently only has `put_u16`/`get_u16`).

**Files:**
- Modify: `src/uwb_frame_802_15_4z.h`, `src/uwb_frame_802_15_4z.c`
- Test: `tests/uwb_frame/test_uwb_frame.c`

**Interfaces:**
- Produces: macros `UWB_FRAME_TYPE_BEACON/JOIN/GRANT/KEEPALIVE/RELEASE`, `UWB_ADDR_GATEWAY 0x0000`, `UWB_ADDR_UNASSOC 0xFFFE`, `UWB_PROTO_VER 1`, `UWB_FRAME_N_CFP 12`, `UWB_FRAME_N_CAP 4`, frame-length macros below; internal `static uint32_t get_u32(const uint8_t *p)`.

- [ ] **Step 1: Add the test**

Append to `tests/uwb_frame/test_uwb_frame.c` and call `test_new_constants()` from `main`:

```c
static void test_new_constants(void)
{
    CHECK(UWB_FRAME_TYPE_BEACON    == 0xE5);
    CHECK(UWB_FRAME_TYPE_JOIN      == 0xE6);
    CHECK(UWB_FRAME_TYPE_GRANT     == 0xE7);
    CHECK(UWB_FRAME_TYPE_KEEPALIVE == 0xE8);
    CHECK(UWB_FRAME_TYPE_RELEASE   == 0xE9);
    CHECK(UWB_ADDR_GATEWAY == 0x0000);
    CHECK(UWB_ADDR_UNASSOC == 0xFFFE);
    CHECK(UWB_FRAME_N_CFP  == 12);
    CHECK(UWB_FRAME_LEN_BEACON == 15 + 2 * 12);  /* 39 */
    CHECK(UWB_FRAME_LEN_JOIN   == 19);
    CHECK(UWB_FRAME_LEN_GRANT  == 24);
    CHECK(UWB_FRAME_LEN_KEEPALIVE == 12);
    CHECK(UWB_FRAME_LEN_RELEASE   == 10);
}
```

- [ ] **Step 2: Run test, verify it fails**

Run the Task 0 Step 4 command (swap the `-o` name to `/tmp/t_frame.exe`).
Expected: FAIL to compile — `UWB_FRAME_TYPE_BEACON` undeclared.

- [ ] **Step 3: Add the constants to the header**

In `src/uwb_frame_802_15_4z.h`, after the existing `0xE2/0xE3/0xE4` block:

```c
/* ---- TDMA MAC message types (contract v1) ---- */
#define UWB_FRAME_TYPE_BEACON    0xE5
#define UWB_FRAME_TYPE_JOIN      0xE6
#define UWB_FRAME_TYPE_GRANT     0xE7
#define UWB_FRAME_TYPE_KEEPALIVE 0xE8
#define UWB_FRAME_TYPE_RELEASE   0xE9

#define UWB_ADDR_GATEWAY  0x0000u
#define UWB_ADDR_UNASSOC  0xFFFEu   /* tag src before it is granted a short addr */

#define UWB_PROTO_VER     1
#define UWB_FRAME_N_CFP   12        /* ranging slots per superframe (v1) */
#define UWB_FRAME_N_CAP   4         /* CAP Aloha mini-slots (v1) */

#define UWB_FRAME_LEN_BEACON     (15 + 2 * UWB_FRAME_N_CFP)
#define UWB_FRAME_LEN_JOIN       19
#define UWB_FRAME_LEN_GRANT      24
#define UWB_FRAME_LEN_KEEPALIVE  12
#define UWB_FRAME_LEN_RELEASE    10
#define UWB_FRAME_EUI_LEN        8
```

- [ ] **Step 4: Add the u32 reader to the .c**

In `src/uwb_frame_802_15_4z.c`, next to `get_u16`:

```c
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
```

> If `get_u16`/`put_u16` are already `static` and unused-by-some-path, add `__attribute__((unused))` only if `-Werror` complains; otherwise leave them.

- [ ] **Step 5: Run test, verify it passes**

Run the host build/run command. Expected: no `FAIL`, `exit=0`.

- [ ] **Step 6: Commit**

```bash
git add src/uwb_frame_802_15_4z.h src/uwb_frame_802_15_4z.c tests/uwb_frame/test_uwb_frame.c
git commit -m "feat(uwb): MAC frame constants + u32 field helpers"
```

---

### Task 2: BEACON build / parse / validate + slot map

**Files:**
- Modify: `src/uwb_frame_802_15_4z.h`, `src/uwb_frame_802_15_4z.c`
- Test: `tests/uwb_frame/test_uwb_frame.c`

**Interfaces:**
- Produces:
  - `int uwb_frame_beacon_build(uint8_t *buf, size_t buf_len, uint32_t frame_counter, const uint16_t *slot_map, uint8_t n_slots);`
  - `int uwb_frame_parse_beacon(const uint8_t *buf, size_t len, uint8_t *proto_ver, uint32_t *frame_counter, uint16_t *slot_map_out, uint8_t *n_slots);`
  - `bool uwb_frame_is_beacon(const uint8_t *buf, size_t len);`
  - `int uwb_frame_beacon_find_addr(const uint16_t *slot_map, uint8_t n_slots, uint16_t addr);` → slot index or `-1`.

- [ ] **Step 1: Write the failing test**

```c
static void test_beacon(void)
{
    uint16_t map[UWB_FRAME_N_CFP] = {0};
    for (int i = 0; i < UWB_FRAME_N_CFP; i++) map[i] = 0xFFFF;  /* all idle */
    map[3] = 0x1234;  /* our slot */

    uint8_t buf[64];
    int n = uwb_frame_beacon_build(buf, sizeof(buf), 0xAABBCCDD, map, UWB_FRAME_N_CFP);
    CHECK(n == UWB_FRAME_LEN_BEACON);
    CHECK(buf[5] == 0xFF && buf[6] == 0xFF);       /* dest broadcast */
    CHECK(buf[7] == 0x00 && buf[8] == 0x00);       /* src gateway */
    CHECK(buf[9] == UWB_FRAME_TYPE_BEACON);
    CHECK(buf[10] == UWB_PROTO_VER);
    CHECK(buf[11] == 0xDD && buf[14] == 0xAA);     /* counter LE */
    CHECK(uwb_frame_is_beacon(buf, n));

    uint8_t ver, ns; uint32_t fc; uint16_t out[UWB_FRAME_N_CFP];
    CHECK(uwb_frame_parse_beacon(buf, n, &ver, &fc, out, &ns) == 0);
    CHECK(ver == UWB_PROTO_VER && fc == 0xAABBCCDD && ns == UWB_FRAME_N_CFP);
    CHECK(out[3] == 0x1234 && out[0] == 0xFFFF);
    CHECK(uwb_frame_beacon_find_addr(out, ns, 0x1234) == 3);
    CHECK(uwb_frame_beacon_find_addr(out, ns, 0x9999) == -1);

    CHECK(uwb_frame_beacon_build(buf, 5, 0, map, UWB_FRAME_N_CFP) == -EMSGSIZE);
}
```

- [ ] **Step 2: Run test, verify it fails** — `undefined reference to uwb_frame_beacon_build`.

- [ ] **Step 3: Declare in header** (after the existing builder/parser declarations):

```c
int  uwb_frame_beacon_build(uint8_t *buf, size_t buf_len, uint32_t frame_counter,
                            const uint16_t *slot_map, uint8_t n_slots);
int  uwb_frame_parse_beacon(const uint8_t *buf, size_t len, uint8_t *proto_ver,
                            uint32_t *frame_counter, uint16_t *slot_map_out,
                            uint8_t *n_slots);
bool uwb_frame_is_beacon(const uint8_t *buf, size_t len);
int  uwb_frame_beacon_find_addr(const uint16_t *slot_map, uint8_t n_slots, uint16_t addr);
```

- [ ] **Step 4: Implement in .c**

```c
/* Common header writer (matches the module's existing layout). */
static int write_hdr(uint8_t *buf, size_t buf_len, size_t need,
                     uint16_t dest, uint16_t src, uint8_t type)
{
    if (!buf) return -EINVAL;
    if (buf_len < need) return -EMSGSIZE;
    buf[OFF_FC0] = 0x41; buf[OFF_FC1] = 0x88; buf[OFF_SEQ] = 0;
    put_u16(&buf[OFF_PAN], UWB_FRAME_PANID);
    put_u16(&buf[OFF_DEST], dest);
    put_u16(&buf[OFF_SRC], src);
    buf[OFF_TYPE] = type;
    return 0;
}

int uwb_frame_beacon_build(uint8_t *buf, size_t buf_len, uint32_t frame_counter,
                           const uint16_t *slot_map, uint8_t n_slots)
{
    size_t need = 15u + 2u * (size_t)n_slots;
    int rc = write_hdr(buf, buf_len, need, UWB_FRAME_ADDR_BCAST,
                       UWB_ADDR_GATEWAY, UWB_FRAME_TYPE_BEACON);
    if (rc) return rc;
    if (!slot_map) return -EINVAL;
    buf[10] = UWB_PROTO_VER;
    put_u32(&buf[11], frame_counter);
    for (uint8_t i = 0; i < n_slots; i++) put_u16(&buf[15 + 2 * i], slot_map[i]);
    return (int)need;
}

bool uwb_frame_is_beacon(const uint8_t *buf, size_t len)
{
    return len >= 15 && uwb_frame_is_valid(buf, len) && buf[OFF_TYPE] == UWB_FRAME_TYPE_BEACON;
}

int uwb_frame_parse_beacon(const uint8_t *buf, size_t len, uint8_t *proto_ver,
                           uint32_t *frame_counter, uint16_t *slot_map_out, uint8_t *n_slots)
{
    if (!uwb_frame_is_beacon(buf, len)) return -EINVAL;
    if (((len - 15) % 2) != 0) return -EINVAL;
    uint8_t ns = (uint8_t)((len - 15) / 2);
    if (proto_ver) *proto_ver = buf[10];
    if (frame_counter) *frame_counter = get_u32(&buf[11]);
    if (slot_map_out) for (uint8_t i = 0; i < ns; i++) slot_map_out[i] = get_u16(&buf[15 + 2 * i]);
    if (n_slots) *n_slots = ns;
    return 0;
}

int uwb_frame_beacon_find_addr(const uint16_t *slot_map, uint8_t n_slots, uint16_t addr)
{
    for (uint8_t i = 0; i < n_slots; i++) if (slot_map[i] == addr) return (int)i;
    return -1;
}
```

> `OFF_*` and `UWB_FRAME_PANID` already exist in the module. Reuse them — do not redefine.

- [ ] **Step 5: Run test, verify it passes** — no `FAIL`, `exit=0`.

- [ ] **Step 6: Commit**

```bash
git add src/uwb_frame_802_15_4z.h src/uwb_frame_802_15_4z.c tests/uwb_frame/test_uwb_frame.c
git commit -m "feat(uwb): BEACON frame build/parse + slot-map lookup"
```

---

### Task 3: JOIN_REQ and GRANT build / parse

**Files:** Modify `src/uwb_frame_802_15_4z.{h,c}`; test `tests/uwb_frame/test_uwb_frame.c`.

**Interfaces:**
- Produces:
  - `int uwb_frame_join_build(uint8_t *buf, size_t buf_len, const uint8_t eui[8], uint8_t req_tier);`
  - `int uwb_frame_parse_join(const uint8_t *buf, size_t len, uint8_t eui_out[8], uint8_t *req_tier);`
  - `int uwb_frame_grant_build(uint8_t *buf, size_t buf_len, const uint8_t eui[8], uint16_t short_addr, uint8_t slot_index, uint8_t rate_tier, uint16_t lease);`
  - `int uwb_frame_parse_grant(const uint8_t *buf, size_t len, uint8_t eui_out[8], uint16_t *short_addr, uint8_t *slot_index, uint8_t *rate_tier, uint16_t *lease);`
  - `bool uwb_frame_is_join(const uint8_t*, size_t);` `bool uwb_frame_is_grant(const uint8_t*, size_t);`

- [ ] **Step 1: Write the failing test**

```c
static void test_join_grant(void)
{
    const uint8_t eui[8] = {1,2,3,4,5,6,7,8};
    uint8_t buf[32];

    int n = uwb_frame_join_build(buf, sizeof(buf), eui, 2 /*FAST*/);
    CHECK(n == UWB_FRAME_LEN_JOIN);
    CHECK(buf[5] == 0x00 && buf[6] == 0x00);       /* dest gateway */
    CHECK(buf[7] == 0xFE && buf[8] == 0xFF);       /* src unassoc LE */
    CHECK(buf[9] == UWB_FRAME_TYPE_JOIN);
    CHECK(uwb_frame_is_join(buf, n));
    uint8_t e2[8], t; CHECK(uwb_frame_parse_join(buf, n, e2, &t) == 0);
    CHECK(memcmp(e2, eui, 8) == 0 && t == 2);

    n = uwb_frame_grant_build(buf, sizeof(buf), eui, 0x0007, 3, 1, 50);
    CHECK(n == UWB_FRAME_LEN_GRANT);
    CHECK(buf[5] == 0xFF && buf[6] == 0xFF);       /* dest broadcast (EUI-matched) */
    CHECK(buf[9] == UWB_FRAME_TYPE_GRANT);
    CHECK(uwb_frame_is_grant(buf, n));
    uint8_t e3[8]; uint16_t sa, ls; uint8_t si, rt;
    CHECK(uwb_frame_parse_grant(buf, n, e3, &sa, &si, &rt, &ls) == 0);
    CHECK(memcmp(e3, eui, 8) == 0 && sa == 0x0007 && si == 3 && rt == 1 && ls == 50);
}
```

- [ ] **Step 2: Run test, verify it fails.**

- [ ] **Step 3: Declare in header** (the four builders/parsers + two validators above).

- [ ] **Step 4: Implement in .c**

```c
int uwb_frame_join_build(uint8_t *buf, size_t buf_len, const uint8_t eui[8], uint8_t req_tier)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_JOIN, UWB_ADDR_GATEWAY,
                       UWB_ADDR_UNASSOC, UWB_FRAME_TYPE_JOIN);
    if (rc) return rc;
    if (!eui) return -EINVAL;
    memcpy(&buf[10], eui, UWB_FRAME_EUI_LEN);
    buf[18] = req_tier;
    return UWB_FRAME_LEN_JOIN;
}

bool uwb_frame_is_join(const uint8_t *buf, size_t len)
{
    return len == UWB_FRAME_LEN_JOIN && uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_JOIN;
}

int uwb_frame_parse_join(const uint8_t *buf, size_t len, uint8_t eui_out[8], uint8_t *req_tier)
{
    if (!uwb_frame_is_join(buf, len)) return -EINVAL;
    if (eui_out) memcpy(eui_out, &buf[10], UWB_FRAME_EUI_LEN);
    if (req_tier) *req_tier = buf[18];
    return 0;
}

int uwb_frame_grant_build(uint8_t *buf, size_t buf_len, const uint8_t eui[8],
                          uint16_t short_addr, uint8_t slot_index, uint8_t rate_tier, uint16_t lease)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_GRANT, UWB_FRAME_ADDR_BCAST,
                       UWB_ADDR_GATEWAY, UWB_FRAME_TYPE_GRANT);
    if (rc) return rc;
    if (!eui) return -EINVAL;
    memcpy(&buf[10], eui, UWB_FRAME_EUI_LEN);
    put_u16(&buf[18], short_addr);
    buf[20] = slot_index;
    buf[21] = rate_tier;
    put_u16(&buf[22], lease);
    return UWB_FRAME_LEN_GRANT;
}

bool uwb_frame_is_grant(const uint8_t *buf, size_t len)
{
    return len == UWB_FRAME_LEN_GRANT && uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_GRANT;
}

int uwb_frame_parse_grant(const uint8_t *buf, size_t len, uint8_t eui_out[8],
                          uint16_t *short_addr, uint8_t *slot_index, uint8_t *rate_tier, uint16_t *lease)
{
    if (!uwb_frame_is_grant(buf, len)) return -EINVAL;
    if (eui_out)   memcpy(eui_out, &buf[10], UWB_FRAME_EUI_LEN);
    if (short_addr) *short_addr = get_u16(&buf[18]);
    if (slot_index) *slot_index = buf[20];
    if (rate_tier)  *rate_tier = buf[21];
    if (lease)      *lease = get_u16(&buf[22]);
    return 0;
}
```

- [ ] **Step 5: Run test, verify it passes.**
- [ ] **Step 6: Commit** — `feat(uwb): JOIN_REQ and GRANT frames`.

---

### Task 4: KEEPALIVE and RELEASE build / parse

**Files:** Modify `src/uwb_frame_802_15_4z.{h,c}`; test `tests/uwb_frame/test_uwb_frame.c`.

**Interfaces:**
- Produces:
  - `int uwb_frame_keepalive_build(uint8_t *buf, size_t buf_len, uint16_t src_addr, uint8_t req_tier, uint8_t slot_index);`
  - `int uwb_frame_parse_keepalive(const uint8_t *buf, size_t len, uint16_t *src_addr, uint8_t *req_tier, uint8_t *slot_index);`
  - `int uwb_frame_release_build(uint8_t *buf, size_t buf_len, uint16_t src_addr);`
  - `bool uwb_frame_is_keepalive(const uint8_t*, size_t);` `bool uwb_frame_is_release(const uint8_t*, size_t);`

- [ ] **Step 1: Write the failing test**

```c
static void test_keepalive_release(void)
{
    uint8_t buf[16];
    int n = uwb_frame_keepalive_build(buf, sizeof(buf), 0x0007, 2, 3);
    CHECK(n == UWB_FRAME_LEN_KEEPALIVE);
    CHECK(buf[5] == 0x00 && buf[6] == 0x00);       /* dest gateway */
    CHECK(buf[7] == 0x07 && buf[8] == 0x00);       /* src 0x0007 LE */
    CHECK(buf[9] == UWB_FRAME_TYPE_KEEPALIVE);
    CHECK(uwb_frame_is_keepalive(buf, n));
    uint16_t sa; uint8_t rt, si;
    CHECK(uwb_frame_parse_keepalive(buf, n, &sa, &rt, &si) == 0);
    CHECK(sa == 0x0007 && rt == 2 && si == 3);

    n = uwb_frame_release_build(buf, sizeof(buf), 0x0007);
    CHECK(n == UWB_FRAME_LEN_RELEASE);
    CHECK(buf[9] == UWB_FRAME_TYPE_RELEASE);
    CHECK(uwb_frame_is_release(buf, n));
}
```

- [ ] **Step 2: Run test, verify it fails.**
- [ ] **Step 3: Declare in header** (three builders/parser + two validators).
- [ ] **Step 4: Implement in .c**

```c
int uwb_frame_keepalive_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                              uint8_t req_tier, uint8_t slot_index)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_KEEPALIVE, UWB_ADDR_GATEWAY,
                       src_addr, UWB_FRAME_TYPE_KEEPALIVE);
    if (rc) return rc;
    buf[10] = req_tier;
    buf[11] = slot_index;
    return UWB_FRAME_LEN_KEEPALIVE;
}
bool uwb_frame_is_keepalive(const uint8_t *buf, size_t len)
{
    return len == UWB_FRAME_LEN_KEEPALIVE && uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_KEEPALIVE;
}
int uwb_frame_parse_keepalive(const uint8_t *buf, size_t len, uint16_t *src_addr,
                              uint8_t *req_tier, uint8_t *slot_index)
{
    if (!uwb_frame_is_keepalive(buf, len)) return -EINVAL;
    if (src_addr)   *src_addr = uwb_frame_get_src_addr(buf);
    if (req_tier)   *req_tier = buf[10];
    if (slot_index) *slot_index = buf[11];
    return 0;
}
int uwb_frame_release_build(uint8_t *buf, size_t buf_len, uint16_t src_addr)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_RELEASE, UWB_ADDR_GATEWAY,
                       src_addr, UWB_FRAME_TYPE_RELEASE);
    if (rc) return rc;
    return UWB_FRAME_LEN_RELEASE;
}
bool uwb_frame_is_release(const uint8_t *buf, size_t len)
{
    return len == UWB_FRAME_LEN_RELEASE && uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_RELEASE;
}
```

- [ ] **Step 5: Run test, verify it passes.**
- [ ] **Step 6: Commit** — `feat(uwb): KEEPALIVE and RELEASE frames`.

---

### Task 5: `uwb_net` types, init, and tier-cadence helper

The portable MAC core's data model + the pure cadence helper. No FSM transitions yet.

**Files:**
- Create: `src/uwb_net.h`, `src/uwb_net.c`, `tests/uwb_net/test_uwb_net.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: enums `uwb_net_state_t`, `uwb_tier_t`, `uwb_ev_kind_t`; action bit-flags `UWB_ACT_*`; structs `uwb_net_event`, `uwb_net_ctx`; `void uwb_net_init(struct uwb_net_ctx*, const uint8_t eui[8]);`; `bool uwb_tier_due(uwb_tier_t, uint32_t frame_counter);`; `uint32_t uwb_net_handle(struct uwb_net_ctx*, const struct uwb_net_event*);` (defined in Task 6+).

- [ ] **Step 1: Write the failing test**

Create `tests/uwb_net/test_uwb_net.c`:

```c
#include "uwb_net.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static const uint8_t EUI[8] = {0xDE,0xAD,0xBE,0xEF,0,0,0,1};

static void test_init_and_cadence(void)
{
    struct uwb_net_ctx c;
    uwb_net_init(&c, EUI);
    CHECK(c.state == UWB_ST_SCAN);
    CHECK(c.req_tier == UWB_TIER_FAST);
    CHECK(memcmp(c.eui, EUI, 8) == 0);

    /* FAST: every superframe. */
    CHECK(uwb_tier_due(UWB_TIER_FAST, 0));
    CHECK(uwb_tier_due(UWB_TIER_FAST, 7));
    /* SLOW: every 5. */
    CHECK(uwb_tier_due(UWB_TIER_SLOW, 10));
    CHECK(!uwb_tier_due(UWB_TIER_SLOW, 11));
    /* IDLE: every 25. */
    CHECK(uwb_tier_due(UWB_TIER_IDLE, 50));
    CHECK(!uwb_tier_due(UWB_TIER_IDLE, 51));
}

int main(void)
{
    test_init_and_cadence();
    printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
```

- [ ] **Step 2: Run it, verify it fails** (no `uwb_net.h`):

```bash
GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/uwb_net/test_uwb_net.c src/uwb_net.c -o /tmp/t_net.exe && /tmp/t_net.exe; echo "exit=$?"
```
Expected: FAIL — `uwb_net.h: No such file`.

- [ ] **Step 3: Create `src/uwb_net.h`**

```c
#ifndef UWB_NET_H
#define UWB_NET_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Lifecycle/config constants (contract v1). */
#define UWB_NET_LEASE_SF        50   /* lease length, superframes */
#define UWB_NET_MISS_MAX        3    /* M: consecutive beacon misses -> lost */
#define UWB_NET_JOIN_RETRY_MAX  4    /* N: join attempts -> back to scan */
#define UWB_NET_MIN_ANCHORS     3    /* need >=3 for a 2D fix */
#define UWB_NET_PROTO_VER       1

typedef enum { UWB_TIER_IDLE = 0, UWB_TIER_SLOW = 1, UWB_TIER_FAST = 2 } uwb_tier_t;
typedef enum { UWB_ST_SCAN = 0, UWB_ST_JOINING, UWB_ST_DISCOVER, UWB_ST_RANGING } uwb_net_state_t;

typedef enum {
    UWB_EV_BEACON = 0,   /* valid beacon received this superframe */
    UWB_EV_BEACON_MISS,  /* no valid beacon this superframe */
    UWB_EV_GRANT,        /* a grant addressed to us (EUI matched) */
    UWB_EV_GRANT_MISS,   /* CAP window elapsed, no grant */
    UWB_EV_DISCOVERED,   /* discovery sweep completed */
    UWB_EV_SWEPT,        /* a ranging sweep completed */
    UWB_EV_MOTION        /* motion module reports a tier request */
} uwb_ev_kind_t;

struct uwb_net_event {
    uwb_ev_kind_t kind;
    /* BEACON */
    uint8_t  proto_ver;
    uint32_t frame_counter;
    bool     in_map;     /* our short addr present in the slot map */
    uint8_t  map_slot;   /* slot index we occupy (valid iff in_map) */
    /* GRANT */
    uint16_t g_short_addr;
    uint8_t  g_slot;
    uint8_t  g_tier;
    uint16_t g_lease;
    /* DISCOVERED / SWEPT */
    uint8_t  n_anchors;
    /* MOTION */
    uint8_t  req_tier;
};

/* Action bit-flags returned by uwb_net_handle (a superframe may need >1). */
#define UWB_ACT_NONE            0u
#define UWB_ACT_SEND_JOIN       (1u << 0)
#define UWB_ACT_SEND_KEEPALIVE  (1u << 1)
#define UWB_ACT_RUN_DISCOVER    (1u << 2)
#define UWB_ACT_RUN_SWEEP       (1u << 3)
#define UWB_ACT_SLEEP           (1u << 4)
#define UWB_ACT_TO_SCAN         (1u << 5)   /* lease lost this superframe */

struct uwb_net_ctx {
    uwb_net_state_t state;
    uint8_t   eui[8];
    uint16_t  short_addr;
    uint8_t   slot_index;
    uwb_tier_t tier;            /* granted tier */
    uwb_tier_t req_tier;        /* motion-driven request */
    uint16_t  lease_remaining;  /* superframes until expiry */
    uint8_t   miss_count;       /* consecutive beacon misses */
    uint8_t   join_retries;
    uint32_t  frame_counter;    /* last beacon's counter (cadence ref) */
    uint8_t   n_anchors;        /* selected anchors after discovery */
};

void     uwb_net_init(struct uwb_net_ctx *c, const uint8_t eui[8]);
bool     uwb_tier_due(uwb_tier_t tier, uint32_t frame_counter);
uint32_t uwb_net_handle(struct uwb_net_ctx *c, const struct uwb_net_event *ev);

#endif /* UWB_NET_H */
```

- [ ] **Step 4: Create `src/uwb_net.c` (init + cadence only)**

```c
#include "uwb_net.h"
#include <string.h>

static uint32_t tier_cadence(uwb_tier_t t)
{
    switch (t) {
    case UWB_TIER_FAST: return 1u;
    case UWB_TIER_SLOW: return 5u;
    default:            return 25u;   /* IDLE */
    }
}

bool uwb_tier_due(uwb_tier_t tier, uint32_t frame_counter)
{
    return (frame_counter % tier_cadence(tier)) == 0u;
}

void uwb_net_init(struct uwb_net_ctx *c, const uint8_t eui[8])
{
    memset(c, 0, sizeof(*c));
    memcpy(c->eui, eui, 8);
    c->state    = UWB_ST_SCAN;
    c->tier     = UWB_TIER_FAST;
    c->req_tier = UWB_TIER_FAST;
}

/* uwb_net_handle is implemented incrementally in Tasks 6-8. Temporary stub: */
uint32_t uwb_net_handle(struct uwb_net_ctx *c, const struct uwb_net_event *ev)
{
    (void)c; (void)ev;
    return UWB_ACT_NONE;
}
```

- [ ] **Step 5: Run test, verify it passes** — `OK`, `exit=0`.

- [ ] **Step 6: Add `src/uwb_net.c` to `CMakeLists.txt`** `target_sources` and commit

```bash
git add src/uwb_net.h src/uwb_net.c tests/uwb_net/test_uwb_net.c CMakeLists.txt
git commit -m "feat(uwb): uwb_net data model + tier-cadence helper"
```

---

### Task 6: FSM — SCAN → JOINING → (retry/give-up)

**Files:** Modify `src/uwb_net.c`; test `tests/uwb_net/test_uwb_net.c`.

**Interfaces:**
- Consumes: `uwb_net_ctx`, `uwb_net_event`, `UWB_ACT_*` (Task 5).
- Produces: real `uwb_net_handle` SCAN + JOINING behaviour.

- [ ] **Step 1: Write the failing tests**

```c
static struct uwb_net_event ev_beacon(uint32_t fc, bool in_map, uint8_t slot)
{
    struct uwb_net_event e; memset(&e, 0, sizeof(e));
    e.kind = UWB_EV_BEACON; e.proto_ver = UWB_NET_PROTO_VER;
    e.frame_counter = fc; e.in_map = in_map; e.map_slot = slot;
    return e;
}

static void test_scan_join(void)
{
    struct uwb_net_ctx c; uwb_net_init(&c, EUI);

    /* Wrong proto version: ignored, stays SCAN. */
    struct uwb_net_event bad = ev_beacon(1, false, 0); bad.proto_ver = 99;
    CHECK(uwb_net_handle(&c, &bad) == UWB_ACT_NONE);
    CHECK(c.state == UWB_ST_SCAN);

    /* Valid beacon: -> JOINING, emit join. */
    struct uwb_net_event b = ev_beacon(5, false, 0);
    CHECK(uwb_net_handle(&c, &b) == UWB_ACT_SEND_JOIN);
    CHECK(c.state == UWB_ST_JOINING);

    /* GRANT_MISS retries up to N, then back to SCAN. */
    struct uwb_net_event miss; memset(&miss, 0, sizeof(miss)); miss.kind = UWB_EV_GRANT_MISS;
    for (int i = 1; i < UWB_NET_JOIN_RETRY_MAX; i++)
        CHECK(uwb_net_handle(&c, &miss) == UWB_ACT_SEND_JOIN);
    CHECK(uwb_net_handle(&c, &miss) == UWB_ACT_NONE);   /* Nth failure */
    CHECK(c.state == UWB_ST_SCAN);
}
```

- [ ] **Step 2: Run, verify it fails** (stub returns NONE; `state` never leaves SCAN).

- [ ] **Step 3: Replace the stub** in `src/uwb_net.c`:

```c
uint32_t uwb_net_handle(struct uwb_net_ctx *c, const struct uwb_net_event *ev)
{
    /* Motion is orthogonal: it only updates the requested tier. */
    if (ev->kind == UWB_EV_MOTION) {
        c->req_tier = (uwb_tier_t)ev->req_tier;
        return UWB_ACT_NONE;
    }

    switch (c->state) {
    case UWB_ST_SCAN:
        if (ev->kind == UWB_EV_BEACON && ev->proto_ver == UWB_NET_PROTO_VER) {
            c->frame_counter = ev->frame_counter;
            c->miss_count = 0;
            c->join_retries = 0;
            c->state = UWB_ST_JOINING;
            return UWB_ACT_SEND_JOIN;
        }
        return UWB_ACT_NONE;

    case UWB_ST_JOINING:
        if (ev->kind == UWB_EV_GRANT_MISS) {
            if (++c->join_retries >= UWB_NET_JOIN_RETRY_MAX) {
                c->state = UWB_ST_SCAN;
                return UWB_ACT_NONE;
            }
            return UWB_ACT_SEND_JOIN;
        }
        if (ev->kind == UWB_EV_BEACON) {
            c->miss_count = 0;
            c->frame_counter = ev->frame_counter;
        }
        return UWB_ACT_NONE;   /* GRANT handled in Task 7 */

    default:
        return UWB_ACT_NONE;
    }
}
```

- [ ] **Step 4: Run, verify it passes** (add `test_scan_join();` to `main`).
- [ ] **Step 5: Commit** — `feat(uwb): FSM scan/join transitions`.

---

### Task 7: FSM — JOINING → DISCOVER (grant) and DISCOVER → RANGING

**Files:** Modify `src/uwb_net.c`; test `tests/uwb_net/test_uwb_net.c`.

**Interfaces:**
- Consumes: Task 6 handler.
- Produces: GRANT handling in JOINING; full DISCOVER state.

- [ ] **Step 1: Write the failing tests**

```c
static struct uwb_net_event ev_grant(uint16_t sa, uint8_t slot, uint8_t tier, uint16_t lease)
{
    struct uwb_net_event e; memset(&e, 0, sizeof(e));
    e.kind = UWB_EV_GRANT; e.g_short_addr = sa; e.g_slot = slot;
    e.g_tier = tier; e.g_lease = lease;
    return e;
}

static void test_grant_discover(void)
{
    struct uwb_net_ctx c; uwb_net_init(&c, EUI);
    struct uwb_net_event b = ev_beacon(0, false, 0);
    uwb_net_handle(&c, &b);                         /* -> JOINING */

    struct uwb_net_event g = ev_grant(0x0007, 3, UWB_TIER_FAST, 50);
    CHECK(uwb_net_handle(&c, &g) == UWB_ACT_RUN_DISCOVER);
    CHECK(c.state == UWB_ST_DISCOVER);
    CHECK(c.short_addr == 0x0007 && c.slot_index == 3);
    CHECK(c.tier == UWB_TIER_FAST && c.lease_remaining == 50);

    /* Discovery finds too few anchors -> retry. */
    struct uwb_net_event d; memset(&d, 0, sizeof(d));
    d.kind = UWB_EV_DISCOVERED; d.n_anchors = 2;
    CHECK(uwb_net_handle(&c, &d) == UWB_ACT_RUN_DISCOVER);
    CHECK(c.state == UWB_ST_DISCOVER);

    /* Enough anchors -> RANGING. */
    d.n_anchors = 4;
    CHECK(uwb_net_handle(&c, &d) == UWB_ACT_NONE);
    CHECK(c.state == UWB_ST_RANGING && c.n_anchors == 4);

    /* Lease reclaimed mid-DISCOVER (addr absent) -> SCAN. */
    struct uwb_net_ctx c2; uwb_net_init(&c2, EUI);
    uwb_net_handle(&c2, &b); uwb_net_handle(&c2, &g);   /* DISCOVER */
    struct uwb_net_event lost = ev_beacon(1, false, 0); /* in_map = false */
    CHECK(uwb_net_handle(&c2, &lost) == UWB_ACT_TO_SCAN);
    CHECK(c2.state == UWB_ST_SCAN);
}
```

- [ ] **Step 2: Run, verify it fails.**

- [ ] **Step 3: Extend the handler** — add GRANT to the JOINING case (before its `return`) and a new DISCOVER case:

```c
        if (ev->kind == UWB_EV_GRANT) {       /* inside UWB_ST_JOINING */
            c->short_addr      = ev->g_short_addr;
            c->slot_index      = ev->g_slot;
            c->tier            = (uwb_tier_t)ev->g_tier;
            c->lease_remaining = ev->g_lease;
            c->miss_count      = 0;
            c->state           = UWB_ST_DISCOVER;
            return UWB_ACT_RUN_DISCOVER;
        }
```

```c
    case UWB_ST_DISCOVER:
        if (ev->kind == UWB_EV_DISCOVERED) {
            c->n_anchors = ev->n_anchors;
            if (ev->n_anchors >= UWB_NET_MIN_ANCHORS) {
                c->state = UWB_ST_RANGING;
                return UWB_ACT_NONE;
            }
            return UWB_ACT_RUN_DISCOVER;    /* retry */
        }
        if (ev->kind == UWB_EV_BEACON_MISS) {
            if (++c->miss_count >= UWB_NET_MISS_MAX) {
                c->state = UWB_ST_SCAN; c->miss_count = 0;
                return UWB_ACT_TO_SCAN;
            }
            return UWB_ACT_NONE;
        }
        if (ev->kind == UWB_EV_BEACON) {
            c->miss_count = 0;
            c->frame_counter = ev->frame_counter;
            if (!ev->in_map) {             /* gateway reclaimed our seat */
                c->state = UWB_ST_SCAN;
                return UWB_ACT_TO_SCAN;
            }
        }
        return UWB_ACT_NONE;
```

- [ ] **Step 4: Run, verify it passes** (add `test_grant_discover();` to `main`).
- [ ] **Step 5: Commit** — `feat(uwb): FSM grant + discover transitions`.

---

### Task 8: FSM — RANGING superframe loop (sweep, keepalive, loss)

The behavioural core: per-beacon, detect loss first, keepalive at half-lease, sweep when the tier is due — and never transmit on a missed beacon.

**Files:** Modify `src/uwb_net.c`; test `tests/uwb_net/test_uwb_net.c`.

**Interfaces:**
- Consumes: Task 7 handler.
- Produces: RANGING case. Action flags may combine (`KEEPALIVE | RUN_SWEEP`).

- [ ] **Step 1: Write the failing tests**

```c
/* Helper: drive a fresh ctx into RANGING with the given tier. */
static void to_ranging(struct uwb_net_ctx *c, uwb_tier_t tier)
{
    uwb_net_init(c, EUI);
    struct uwb_net_event b = ev_beacon(0, false, 0); uwb_net_handle(c, &b);
    struct uwb_net_event g = ev_grant(0x0007, 3, tier, UWB_NET_LEASE_SF); uwb_net_handle(c, &g);
    struct uwb_net_event d; memset(&d, 0, sizeof(d)); d.kind = UWB_EV_DISCOVERED; d.n_anchors = 4;
    uwb_net_handle(c, &d);   /* -> RANGING */
}

static void test_ranging(void)
{
    struct uwb_net_ctx c; to_ranging(&c, UWB_TIER_FAST);

    /* FAST beacon, lease healthy, in map, due -> sweep + sleep, no keepalive. */
    struct uwb_net_event b = ev_beacon(2, true, 3);
    uint32_t a = uwb_net_handle(&c, &b);
    CHECK(a & UWB_ACT_RUN_SWEEP);
    CHECK(!(a & UWB_ACT_SEND_KEEPALIVE));
    CHECK(c.slot_index == 3);

    /* SLOW tier, frame_counter not a multiple of 5 -> sleep, no sweep. */
    struct uwb_net_ctx cs; to_ranging(&cs, UWB_TIER_SLOW);
    struct uwb_net_event b1 = ev_beacon(1, true, 3);
    CHECK(uwb_net_handle(&cs, &b1) == UWB_ACT_SLEEP);
    struct uwb_net_event b5 = ev_beacon(5, true, 3);
    CHECK(uwb_net_handle(&cs, &b5) & UWB_ACT_RUN_SWEEP);

    /* Lease decays to half -> keepalive flag set, lease renewed. */
    struct uwb_net_ctx ck; to_ranging(&ck, UWB_TIER_FAST);
    ck.lease_remaining = UWB_NET_LEASE_SF / 2;     /* at threshold */
    struct uwb_net_event bk = ev_beacon(2, true, 3);
    CHECK(uwb_net_handle(&ck, &bk) & UWB_ACT_SEND_KEEPALIVE);
    CHECK(ck.lease_remaining == UWB_NET_LEASE_SF);  /* renewed optimistically */

    /* Beacon miss x M -> SCAN, never a TX action. */
    struct uwb_net_ctx cm; to_ranging(&cm, UWB_TIER_FAST);
    struct uwb_net_event miss; memset(&miss, 0, sizeof(miss)); miss.kind = UWB_EV_BEACON_MISS;
    CHECK(uwb_net_handle(&cm, &miss) == UWB_ACT_NONE);
    CHECK(uwb_net_handle(&cm, &miss) == UWB_ACT_NONE);
    CHECK(uwb_net_handle(&cm, &miss) == UWB_ACT_TO_SCAN);
    CHECK(cm.state == UWB_ST_SCAN);

    /* Addr absent from map -> SCAN immediately. */
    struct uwb_net_ctx cr; to_ranging(&cr, UWB_TIER_FAST);
    struct uwb_net_event gone = ev_beacon(2, false, 0);
    CHECK(uwb_net_handle(&cr, &gone) == UWB_ACT_TO_SCAN);
    CHECK(cr.state == UWB_ST_SCAN);

    /* Sweep returns too few anchors -> re-discover. */
    struct uwb_net_ctx cd; to_ranging(&cd, UWB_TIER_FAST);
    struct uwb_net_event sw; memset(&sw, 0, sizeof(sw)); sw.kind = UWB_EV_SWEPT; sw.n_anchors = 2;
    CHECK(uwb_net_handle(&cd, &sw) == UWB_ACT_RUN_DISCOVER);
    CHECK(cd.state == UWB_ST_DISCOVER);
}
```

- [ ] **Step 2: Run, verify it fails.**

- [ ] **Step 3: Add the RANGING case** to the handler:

```c
    case UWB_ST_RANGING:
        if (ev->kind == UWB_EV_BEACON_MISS) {
            if (++c->miss_count >= UWB_NET_MISS_MAX) {
                c->state = UWB_ST_SCAN; c->miss_count = 0;
                return UWB_ACT_TO_SCAN;
            }
            return UWB_ACT_NONE;            /* never TX on a missed beacon */
        }
        if (ev->kind == UWB_EV_SWEPT) {
            if (ev->n_anchors < UWB_NET_MIN_ANCHORS) {
                c->state = UWB_ST_DISCOVER;
                return UWB_ACT_RUN_DISCOVER;
            }
            return UWB_ACT_NONE;
        }
        if (ev->kind == UWB_EV_BEACON) {
            c->miss_count = 0;
            c->frame_counter = ev->frame_counter;
            if (!ev->in_map) {              /* lease reclaimed */
                c->state = UWB_ST_SCAN;
                return UWB_ACT_TO_SCAN;
            }
            c->slot_index = ev->map_slot;
            if (c->lease_remaining > 0) c->lease_remaining--;

            uint32_t act = 0;
            if (c->lease_remaining <= (UWB_NET_LEASE_SF / 2)) {
                act |= UWB_ACT_SEND_KEEPALIVE;
                c->lease_remaining = UWB_NET_LEASE_SF;   /* optimistic renew */
            }
            if (uwb_tier_due(c->tier, c->frame_counter)) {
                act |= UWB_ACT_RUN_SWEEP;
            } else {
                act |= UWB_ACT_SLEEP;
            }
            return act;
        }
        return UWB_ACT_NONE;
```

- [ ] **Step 4: Run, verify it passes** (add `test_ranging();` to `main`).
- [ ] **Step 5: Self-check the whole FSM suite** — re-run `/tmp/t_net.exe`; expect `OK`.
- [ ] **Step 6: Commit** — `feat(uwb): FSM ranging superframe loop`.

---

### Task 9: Target radio_ops — DW3000 runner

Wire the pure FSM to the radio. This is target-only glue (not host-tested); it reuses today's ranging primitives and the IRQ/semaphore model from `uwb_ss_initiator.c`.

**Files:**
- Create: `src/uwb_radio_ops.h` (the seam), `src/uwb_net_runner.c` (the thread that pumps FSM events ↔ radio)
- Modify: `src/uwb_ss_initiator.c` (expose `do_one_range_anchor` + frame helpers; remove the free-running cadence loop), `CMakeLists.txt`

**Interfaces:**
- Consumes: `uwb_net_handle`, all `uwb_frame_*` builders/parsers, `pos_solve`.
- Produces: `void uwb_net_runner_start(const uint8_t eui[8]);`, `void uwb_net_set_tier(uwb_tier_t);` (called from motion).

- [ ] **Step 1: Define the radio seam** — create `src/uwb_radio_ops.h`:

```c
#ifndef UWB_RADIO_OPS_H
#define UWB_RADIO_OPS_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pos_solver.h"   /* struct pos_meas */

/* Blocking RX of one beacon; returns frame length or <0 on timeout. */
int  uwb_radio_rx_beacon(uint8_t *buf, size_t buf_len, uint32_t timeout_ms);
/* TX one frame in a CAP mini-slot (caller applies Aloha backoff via slot arg). */
int  uwb_radio_tx_cap(const uint8_t *buf, size_t len, uint8_t minislot);
/* Run discovery; fill anchor coords/ids; return anchor count. */
int  uwb_radio_discover(struct pos_meas *out, size_t max);
/* Range the previously discovered anchors; fill ranges; return count. */
int  uwb_radio_sweep(struct pos_meas *out, size_t max);
/* Sleep the DW3000 until `wake_ms` (monotonic). */
void uwb_radio_sleep_until(uint32_t wake_ms);
uint32_t uwb_radio_now_ms(void);
#endif
```

- [ ] **Step 2: Refactor `uwb_ss_initiator.c`** — make `do_one_range_anchor()` non-static and declared in a header (e.g. add a prototype to `uwb_ss_initiator.h`), and **delete** the free-running `ss_twr_fn` cadence loop body (the `while(1)` with `range_tick`/`RNG_FAST_MS`). Keep `do_one_range_anchor`, `position_publish`, `twr_msgq`, `ble_tx_fn`, and the cal path. The runner (next step) becomes the new owner of the ranging thread.

- [ ] **Step 3: Implement `src/uwb_net_runner.c`** — one thread that each superframe: `uwb_radio_rx_beacon` → build a `uwb_net_event` (parse beacon, set `in_map`/`map_slot` via `uwb_frame_parse_beacon` + `uwb_frame_beacon_find_addr`) → `uwb_net_handle` → execute returned flags:
  - `SEND_JOIN` → `uwb_frame_join_build` + `uwb_radio_tx_cap`; then `rx` for a grant, feed `UWB_EV_GRANT`/`UWB_EV_GRANT_MISS`.
  - `SEND_KEEPALIVE` → `uwb_frame_keepalive_build` + `uwb_radio_tx_cap`.
  - `RUN_DISCOVER` → `uwb_radio_discover` → feed `UWB_EV_DISCOVERED`.
  - `RUN_SWEEP` → `uwb_radio_sleep_until(slot_start)` then `uwb_radio_sweep` → `pos_solve` → `position_publish`; feed `UWB_EV_SWEPT`.
  - `SLEEP`/none → `uwb_radio_sleep_until(next_beacon)`.
  Compute `slot_start` per spec §4: `t0 + T_beacon + g + N_CAP·t_minislot + g + slot_index·(T_slot+g)`; put the v1 timing constants in `uwb_net_runner.c` as `#define`s matching the contract.
  Implement the `uwb_radio_*` functions in this file (or `uwb_radio_ops_dw3000.c`) using the existing `dwt_*` calls, `port_*` IRQ helpers, `wait_event()` pattern, and `dwt_entersleep()`/wake for deep sleep.

- [ ] **Step 4: Provide `uwb_net_set_tier`** — stores the requested tier and, on the next beacon, the runner feeds a `UWB_EV_MOTION` event so the FSM updates `req_tier` (reported in the next keepalive).

- [ ] **Step 5: Add both files to `CMakeLists.txt`; build the firmware** (user builds/flashes per CLAUDE.md). Confirm it compiles.

- [ ] **Step 6: Commit** — `feat(uwb): DW3000 radio_ops runner driving the MAC FSM`.

---

### Task 10: Integrate into `main.c` + repurpose motion

**Files:** Modify `src/main.c`, `src/motion.c`, `CMakeLists.txt` (if needed).

**Interfaces:**
- Consumes: `uwb_net_runner_start`, `uwb_net_set_tier`.

- [ ] **Step 1: Replace `uwb_ss_initiator_start()` in `main.c`** with `uwb_net_runner_start(eui)`, where `eui` is read from the DW3000/factory (or a board MAC). Keep the `cal_init()` reporting and LED/BLE setup unchanged. Cal stays a separate bench path (not started concurrently with the runner).

- [ ] **Step 2: Repurpose `uwb_set_moving()` in `motion.c`** — instead of toggling local cadence, map moving→`UWB_TIER_FAST`, still→`UWB_TIER_IDLE` and call `uwb_net_set_tier(...)`. (A short "recently moved" SLOW window can be added later; not required for v1.)

- [ ] **Step 3: Build the firmware.** Confirm it links (watch for the `FP_SOFTABI`/CMSIS-DSP link notes in CLAUDE.md).

- [ ] **Step 4: Bench bring-up checklist** (manual, on hardware): with a gateway-anchor emitting beacons + responders: confirm the tag advertises, joins (LED green on BLE connect), and emits `P:x,y`; confirm `cal` still works against the `WAVE` reference with the runner stopped. Capture a logic-analyzer trace of one superframe to **measure real `T_slot`** and feed it back into the contract's v1 constants.

- [ ] **Step 5: Commit** — `feat(uwb): tag runs TDMA MAC; motion drives rate tier`.

---

## Self-Review

**Spec coverage (contract):** superframe/timing → Task 9 runner constants + Task 10 bench measurement; addressing/framing → Tasks 1–4; function codes → Task 1; BEACON/JOIN/GRANT/KEEPALIVE/RELEASE layouts → Tasks 2–4; lease lifecycle + tiers → Tasks 5,8; loss/handover rules → Tasks 7,8; multi-poll deferral → unchanged (single-poll reused in Task 9); versioning → Task 6 (proto_ver gate in SCAN). **Spec coverage (tag-side):** host/target seam → Tasks 5–9 (pure FSM) + Task 9 (`radio_ops`); state machine → Tasks 6–8; per-superframe loop → Task 9; error/power → Tasks 8,9; cal coexistence → Task 10; test strategy → host tests Tasks 1–8, bench Task 10.

**Placeholder scan:** Task 9 (target glue) and Task 10 (integration/bench) are described as wiring steps rather than full code, because they depend on hardware behaviour (real `dwt_*` timing, EUI source, slot-time measurement) that is validated on-device, not by host assertions — consistent with the spec marking these target-only. All host-testable logic (Tasks 1–8) has complete code and concrete tests.

**Type consistency:** `uwb_net_handle` returns `uint32_t` action-flags everywhere; `UWB_ACT_TO_SCAN` (added in Task 5 header) is used in Tasks 7–8; event constructors `ev_beacon`/`ev_grant` and `to_ranging` are defined once and reused; frame builder/parser signatures in the Interfaces blocks match their implementations.

---

## Execution Handoff

Plan complete and saved to `plan/2026-06-17-uwb-tdma-mac-tag-implementation.md`. Two execution options:

1. **Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.
2. **Inline Execution** — execute tasks in this session with checkpoints for review.

Which approach?
