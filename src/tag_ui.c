#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <stdio.h>
#include "tag_ui.h"
#include "batt.h"
#include "tag_ui_color.h"
#include "ble_log.h"

/* Button: P0.17, button0 alias in the board DTS. */
static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET(DT_ALIAS(button0), gpios);

static const struct device *strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

#define DEBOUNCE_MS   30U
#define DOUBLE_MS     350U    /* window to detect a second press */
#define BLINK_MS       500U
#define BATTERY_MS    5000U

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    struct led_rgb px = { .r = r, .g = g, .b = b };
    led_strip_update_rgb(strip, &px, 1);
}

#define LED_OFF()     led_set(0, 0, 0)
#define LED_ORANGE()  led_set(10, 4, 0)

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

/* Discrete SoC -> color (matches the design spec). */
static void led_show_battery(void)
{
    int soc;
    int err = batt_read_soc(&soc);
    char msg[20];

    if (err == -EBUSY) {
        ble_log_send("BATT: charging\n");
        log_current();
        led_set(0, 8, 8);           /* cyan: on the charger, no valid SoC */
        return;
    }

    if (err != 0) {
        snprintf(msg, sizeof(msg), "BATT err %d\n", err);
        ble_log_send(msg);
        log_current();
        led_set(0, 0, 10);          /* dim blue: no gauge data */
        return;
    }

    snprintf(msg, sizeof(msg), "BATT: %d%%\n", soc);
    ble_log_send(msg);
    log_current();

    uint8_t r, g, b;
    soc_to_rgb(soc, &r, &g, &b);
    led_set(r, g, b);
}

/* Press timestamps (uptime ms) flow ISR -> UI thread. */
K_MSGQ_DEFINE(press_q, sizeof(uint32_t), 8, 4);

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
    /* Only count the active (pressed) edge. */
    if (gpio_pin_get_dt(&button) != 1) {
        return;
    }
    k_msgq_put(&press_q, &now, K_NO_WAIT);   /* drop if full */
}

#define UI_PRIO    7
#define UI_STACK   1024

K_THREAD_STACK_DEFINE(ui_stack, UI_STACK);
static struct k_thread ui_tid;

static void ui_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    uint32_t t1, t2;

    while (1) {
        /* Wait for the first press of a gesture. */
        if (k_msgq_get(&press_q, &t1, K_FOREVER) != 0) {
            continue;
        }

        /* Wait up to DOUBLE_MS for a second press; presses can sit in the
         * queue while this thread is delayed, so confirm with timestamps. */
        int got = k_msgq_get(&press_q, &t2, K_MSEC(DOUBLE_MS));

        if (got == 0 && (t2 - t1) <= DOUBLE_MS) {
            /* Double press: blink orange until any press; toggle every BLINK_MS. */
            bool on = true;
            while (1) {
                if (on) { LED_ORANGE(); } else { LED_OFF(); }
                on = !on;
                if (k_msgq_get(&press_q, &t2, K_MSEC(BLINK_MS)) == 0) {
                    break;   /* press -> stop blinking; consumed (no battery display) */
                }
            }
            LED_OFF();
        } else {
            /* Single press: battery color for 5 s. */
            led_show_battery();
            if (got != 0) {
                got = k_msgq_get(&press_q, &t2, K_MSEC(BATTERY_MS));
            }
            /* A press during the display (or a stale dequeued one) starts a
             * fresh gesture. */
            if (got == 0) {
                LED_OFF();
                k_msgq_put(&press_q, &t2, K_NO_WAIT);
                continue;
            }
            LED_OFF();
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
