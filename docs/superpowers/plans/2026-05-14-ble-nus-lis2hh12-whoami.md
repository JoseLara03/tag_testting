# BLE NUS Debug Transport + LIS2HH12 WHO_AM_I Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream the LIS2HH12 WHO_AM_I register value over BLE Nordic UART Service to verify the sensor is wired and responding correctly.

**Architecture:** Three focused modules — `ble_log` owns BLE init/advertising/NUS send, `lis2hh12_if` owns the Zephyr I2C callbacks that the PID driver calls through, and `main` sequences init → wait-for-connection → read WHO_AM_I → send result. The STM PID source (`lis2hh12_reg.c`) is compiled directly from the submodule.

**Tech Stack:** Zephyr 4.1 / nCS 3.2.4, BLE NUS (`CONFIG_BT_NUS`), Zephyr I2C API (`i2c_burst_read`/`i2c_burst_write`), STM lis2hh12-pid submodule.

---

## File Map

| Action | Path | Responsibility |
|--------|------|----------------|
| Modify | `prj.conf` | Add BLE + NUS Kconfig |
| Modify | `CMakeLists.txt` | Add new sources + PID include path |
| Create | `src/ble_log.h` | Public API: `ble_log_init`, `ble_log_wait_connected`, `ble_log_send` |
| Create | `src/ble_log.c` | BLE enable, NUS init, advertising, connection tracking, semaphore |
| Create | `src/lis2hh12_if.h` | Public API: `lis2hh12_if_init` |
| Create | `src/lis2hh12_if.c` | `platform_read`/`platform_write` Zephyr I2C callbacks, addr `0x1E` |
| Modify | `src/main.c` | Wire modules, read WHO_AM_I, send result string |

---

## Task 1: Kconfig and CMake

**Files:**
- Modify: `prj.conf`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Update `prj.conf`**

Replace the empty file with:

```
CONFIG_BT=y
CONFIG_BT_HCI=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_DEVICE_NAME="nRF52_Tag"
CONFIG_BT_NUS=y
```

- [ ] **Step 2: Update `CMakeLists.txt`**

Replace the file with:

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(tag_testting)

target_sources(app PRIVATE
    src/main.c
    src/ble_log.c
    src/lis2hh12_if.c
    drivers/lis2hh12-pid/lis2hh12_reg.c
)

target_include_directories(app PRIVATE
    drivers/lis2hh12-pid
)
```

- [ ] **Step 3: Commit**

```bash
git add prj.conf CMakeLists.txt
git commit -m "feat: add BLE NUS Kconfig and wire PID sources in CMake"
```

---

## Task 2: `ble_log` module

**Files:**
- Create: `src/ble_log.h`
- Create: `src/ble_log.c`

- [ ] **Step 1: Create `src/ble_log.h`**

```c
#ifndef BLE_LOG_H
#define BLE_LOG_H

int  ble_log_init(void);
void ble_log_wait_connected(void);
void ble_log_send(const char *msg);

#endif /* BLE_LOG_H */
```

- [ ] **Step 2: Create `src/ble_log.c`**

```c
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <bluetooth/services/nus.h>
#include "ble_log.h"

static struct bt_conn *current_conn;
static K_SEM_DEFINE(conn_sem, 0, 1);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (!err) {
        current_conn = bt_conn_ref(conn);
        k_sem_give(&conn_sem);
    }
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

static void nus_received(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    /* TX-only — ignore incoming data */
}

static struct bt_nus_cb nus_cb = {
    .received = nus_received,
};

int ble_log_init(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        return err;
    }

    err = bt_nus_init(&nus_cb);
    if (err) {
        return err;
    }

    return bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
}

void ble_log_wait_connected(void)
{
    k_sem_take(&conn_sem, K_FOREVER);
}

void ble_log_send(const char *msg)
{
    if (!current_conn) {
        return;
    }
    bt_nus_send(current_conn, (const uint8_t *)msg, strlen(msg));
}
```

- [ ] **Step 3: Commit**

```bash
git add src/ble_log.h src/ble_log.c
git commit -m "feat: add ble_log module (NUS init + send)"
```

---

## Task 3: `lis2hh12_if` module

**Files:**
- Create: `src/lis2hh12_if.h`
- Create: `src/lis2hh12_if.c`

- [ ] **Step 1: Create `src/lis2hh12_if.h`**

```c
#ifndef LIS2HH12_IF_H
#define LIS2HH12_IF_H

#include "lis2hh12_reg.h"

int lis2hh12_if_init(stmdev_ctx_t *ctx);

#endif /* LIS2HH12_IF_H */
```

- [ ] **Step 2: Create `src/lis2hh12_if.c`**

`LIS2HH12_ADDR` is the 7-bit address with SA0=GND.  
`i2c_burst_write(dev, addr, reg, buf, len)` sends `[reg, buf[0]…]` in one transaction.  
`i2c_burst_read(dev, addr, reg, buf, len)` writes `reg` then reads `len` bytes.

```c
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "lis2hh12_if.h"

#define LIS2HH12_ADDR 0x1EU  /* SA0 = GND */

static int32_t platform_write(void *handle, uint8_t reg,
                               const uint8_t *buf, uint16_t len)
{
    return i2c_burst_write((const struct device *)handle,
                           LIS2HH12_ADDR, reg, buf, len);
}

static int32_t platform_read(void *handle, uint8_t reg,
                              uint8_t *buf, uint16_t len)
{
    return i2c_burst_read((const struct device *)handle,
                          LIS2HH12_ADDR, reg, buf, len);
}

int lis2hh12_if_init(stmdev_ctx_t *ctx)
{
    static const struct device *i2c_bus =
        DEVICE_DT_GET(DT_NODELABEL(i2c0));

    if (!device_is_ready(i2c_bus)) {
        return -ENODEV;
    }

    ctx->write_reg = platform_write;
    ctx->read_reg  = platform_read;
    ctx->handle    = (void *)i2c_bus;

    return 0;
}
```

- [ ] **Step 3: Commit**

```bash
git add src/lis2hh12_if.h src/lis2hh12_if.c
git commit -m "feat: add lis2hh12_if module (Zephyr I2C platform callbacks)"
```

---

## Task 4: `main.c` integration

**Files:**
- Modify: `src/main.c`

- [ ] **Step 1: Replace `src/main.c`**

```c
#include <stdio.h>
#include <zephyr/kernel.h>
#include "ble_log.h"
#include "lis2hh12_if.h"
#include "lis2hh12_reg.h"

int main(void)
{
    stmdev_ctx_t dev_ctx = {0};
    uint8_t who_am_i = 0;
    char msg[48];

    ble_log_init();

    if (lis2hh12_if_init(&dev_ctx) != 0) {
        /* I2C bus not ready — nothing we can do without debug output yet */
        k_sleep(K_FOREVER);
        return 0;
    }

    /* Block until a central connects and enables NUS notifications */
    ble_log_wait_connected();

    lis2hh12_dev_id_get(&dev_ctx, &who_am_i);

    if (who_am_i == LIS2HH12_ID) {
        snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X (OK)\r\n", who_am_i);
    } else {
        snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X (FAIL, expected 0x%02X)\r\n",
                 who_am_i, LIS2HH12_ID);
    }

    ble_log_send(msg);

    k_sleep(K_FOREVER);
    return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/main.c
git commit -m "feat: wire BLE NUS + LIS2HH12 WHO_AM_I read in main"
```

---

## Task 5: Build and verify

This task is performed by the user — report any compiler or linker errors.

- [ ] **Step 1: Hand off to user for build**

Ask the user to build with `west build -b nRF52833_tag` and report the output.

- [ ] **Step 2: Flash and test**

Flash the firmware. Open the **nRF Connect** mobile app, scan for `nRF52_Tag`, connect, enable notifications on the NUS TX characteristic, and observe the message.

Expected output in the NUS terminal:
```
WHO_AM_I = 0x41 (OK)
```

If the I2C read fails (all zeros or wrong value), the output will be:
```
WHO_AM_I = 0x00 (FAIL, expected 0x41)
```
which indicates a wiring or address issue.
