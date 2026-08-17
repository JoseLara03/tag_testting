#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <stdio.h>
#include "tag_ui.h"
#include "batt.h"
#include "tag_ui_color.h"
#include "ble_log.h"
#include "tag_alert.h"

/* Button: P0.17, button0 alias in the board DTS. */
static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET(DT_ALIAS(button0), gpios);

static const struct device *strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

#define DEBOUNCE_MS   30U
#define DOUBLE_MS     350U    /* window to detect a second press */
#define LONG_MS      3000U    /* held-down duration that fires a CANCEL */
#define BLINK_MS      500U    /* red-pulse half-period and idle poll rate */
#define BATTERY_MS   5000U
#define GREEN_FLASH_MS 300U

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    struct led_rgb px = { .r = r, .g = g, .b = b };
    led_strip_update_rgb(strip, &px, 1);
}

#define LED_OFF()    led_set(0, 0, 0)
#define LED_RED()    led_set(10, 0, 0)
#define LED_GREEN()  led_set(0, 10, 0)

/* Standing indicator: while a HELP alert is active, pulse red -- this takes
 * priority over the battery readout and over the idle-dark state. Called
 * both from the idle wait loop (so the pulse keeps animating with no button
 * activity) and while a battery readout would otherwise be showing. Returns
 * true if it drove the LED (caller should not also set it). */
static bool led_show_alert(void)
{
    if (!tag_alert_active()) {
        return false;
    }
    if (((k_uptime_get_32() / BLINK_MS) % 2) == 0) {
        LED_RED();
    } else {
        LED_OFF();
    }
    return true;
}

static void led_idle(void)
{
    if (!led_show_alert()) {
        LED_OFF();
    }
}

/* Current draw is measured directly by the PMIC and needs no battery model,
 * so it is reported on every path — including the ones with no percentage. */
static void log_current(void)
{
    int ma;

    if (batt_read_current(&ma) == 0) {
        char imsg[16];
        snprintf(imsg, sizeof(imsg), "I:%dmA\n", ma);
        ble_log_send(imsg);
    }
}

/* Discrete SoC -> color (matches the design spec). The BLE readout is sent
 * regardless of the alert state; the LED colour is skipped when an alert is
 * active, since the standing red pulse takes priority over it. */
static void led_show_battery(void)
{
    int soc;
    int err = batt_read_soc(&soc);
    char msg[20];
    bool alert = tag_alert_active();

    if (err == -EBUSY) {
        ble_log_send("BATT: charging\n");
        log_current();
        if (!alert) {
            led_set(0, 8, 8);           /* cyan: on the charger, no valid SoC */
        }
        return;
    }

    if (err != 0) {
        snprintf(msg, sizeof(msg), "BATT err %d\n", err);
        ble_log_send(msg);
        log_current();
        if (!alert) {
            led_set(0, 0, 10);          /* dim blue: no gauge data */
        }
        return;
    }

    snprintf(msg, sizeof(msg), "BATT: %d%%\n", soc);
    ble_log_send(msg);
    log_current();

    if (!alert) {
        uint8_t r, g, b;
        soc_to_rgb(soc, &r, &g, &b);
        led_set(r, g, b);
    }
}

/* Both button edges flow ISR -> UI thread, timestamped, so the UI thread can
 * measure a hold instead of only reacting to the pressed edge. */
struct btn_ev {
    uint32_t t_ms;
    bool     pressed;
};

K_MSGQ_DEFINE(press_q, sizeof(struct btn_ev), 8, 4);

static struct gpio_callback btn_cb;

static void button_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    static uint32_t last_edge_ms;
    uint32_t now = k_uptime_get_32();
    bool bounced = (now - last_edge_ms) < DEBOUNCE_MS;

    last_edge_ms = now;
    if (bounced) {
        return;
    }

    struct btn_ev ev = {
        .t_ms    = now,
        .pressed = (gpio_pin_get_dt(&button) == 1),
    };
    k_msgq_put(&press_q, &ev, K_NO_WAIT);   /* drop if full */
}

#define UI_PRIO    7
#define UI_STACK   1024

K_THREAD_STACK_DEFINE(ui_stack, UI_STACK);
static struct k_thread ui_tid;

static void ui_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct btn_ev ev1, ev2;

    while (1) {
        /* Idle: wait for the next press, waking periodically to keep the
         * standing alert pulse animating even with no button activity.
         * A lone release edge here (e.g. left over from a prior gesture)
         * is not the start of anything and is discarded. */
        led_idle();
        if (k_msgq_get(&press_q, &ev1, K_MSEC(BLINK_MS)) != 0) {
            continue;
        }
        if (!ev1.pressed) {
            continue;
        }

        uint32_t t1 = ev1.t_ms;

        /* Phase 1: up to DOUBLE_MS for a second PRESS. Deciding from
         * timestamps, not arrival order (the presses can queue up while this
         * thread is delayed) -- see the historical rationale that used to
         * live at tag_ui.c:119-121; multi-edge queuing makes it more true,
         * not less, since a release can now also be sitting in the queue.
         * Releases seen here are consumed, not treated as presses. */
        bool released_first = false;
        bool double_press    = false;

        for (;;) {
            int32_t remain = (int32_t)DOUBLE_MS - (int32_t)(k_uptime_get_32() - t1);
            if (remain <= 0) {
                break;
            }
            if (k_msgq_get(&press_q, &ev2, K_MSEC((uint32_t)remain)) != 0) {
                break;
            }
            if (ev2.pressed) {
                double_press = true;
                break;
            }
            released_first = true;   /* keep waiting out the window */
        }

        if (double_press) {
            /* Double press: raise HELP. No battery display for this gesture;
             * the standing red pulse (driven by tag_alert_active() from the
             * idle loop above) is the feedback from here on. */
            tag_alert_raise();
            continue;
        }

        if (!released_first) {
            /* Still held past DOUBLE_MS with no second press: candidate long
             * press. Bounded wait on the queue (not a blocking wait for the
             * release edge) until LONG_MS total hold time, watching for
             * either the release (-> ordinary single press) or the 3 s mark
             * (-> CANCEL, fired while still held). */
            bool long_press = false;

            for (;;) {
                int32_t remain = (int32_t)LONG_MS - (int32_t)(k_uptime_get_32() - t1);
                if (remain <= 0) {
                    long_press = true;
                    break;
                }
                if (k_msgq_get(&press_q, &ev2, K_MSEC((uint32_t)remain)) != 0) {
                    continue;   /* re-check the deadline */
                }
                if (!ev2.pressed) {
                    released_first = true;
                    break;      /* released before 3 s: ordinary single press */
                }
                /* A press edge while already held is not meaningful here;
                 * consume it and keep waiting out the hold. */
            }

            if (long_press) {
                tag_alert_cancel();
                LED_GREEN();
                k_sleep(K_MSEC(GREEN_FLASH_MS));
                LED_OFF();
                /* Consume the eventual release (and anything else queued
                 * while it was held) before returning to idle, so it is not
                 * mistaken for the start of a fresh gesture. */
                while (k_msgq_get(&press_q, &ev2, K_MSEC(BLINK_MS)) == 0) {
                    if (!ev2.pressed) {
                        break;
                    }
                }
                continue;
            }
        }

        /* Single press: battery colour (unless superseded by an active
         * alert's red pulse) for BATTERY_MS. A press during the display (or
         * a stale dequeued one) starts a fresh gesture, as before. */

        /* The same gesture opens the 60 s BLE advertising window, so the
         * readout gesture already documented on the button is also how the
         * tag is made discoverable -- no separate ritual to remember. Called
         * before the readout: led_show_battery() sends over NUS, and the
         * window needs to be opening while the operator reaches for a phone. */
        ble_log_adv_window();
        led_show_battery();

        uint32_t deadline = k_uptime_get_32() + BATTERY_MS;
        bool restart = false;

        for (;;) {
            int32_t remain = (int32_t)(deadline - k_uptime_get_32());
            if (remain <= 0) {
                break;
            }
            uint32_t wait_ms = (uint32_t)remain;
            if (wait_ms > BLINK_MS) {
                wait_ms = BLINK_MS;
            }
            if (k_msgq_get(&press_q, &ev2, K_MSEC(wait_ms)) == 0) {
                if (ev2.pressed) {
                    restart = true;
                    break;
                }
                continue;   /* release: keep showing the readout */
            }
            /* Timeout of this short slice: re-check the deadline, and
             * refresh the alert pulse in case it takes over mid-display. */
            led_show_alert();
        }

        led_idle();
        if (restart) {
            k_msgq_put(&press_q, &ev2, K_NO_WAIT);
        }
    }
}

void tag_ui_init(void)
{
    if (!gpio_is_ready_dt(&button)) {
        return;
    }
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&btn_cb, button_isr, BIT(button.pin));
    gpio_add_callback(button.port, &btn_cb);

    k_thread_create(&ui_tid, ui_stack, K_THREAD_STACK_SIZEOF(ui_stack),
                    ui_fn, NULL, NULL, NULL, UI_PRIO, 0, K_NO_WAIT);
}
