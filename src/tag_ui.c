#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "tag_ui.h"
#include "ble_log.h"

/* Button: P0.17, button0 alias in the board DTS. */
static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET(DT_ALIAS(button0), gpios);

#define DEBOUNCE_MS   30U
#define DOUBLE_MS     350U    /* window to detect a second press */

/* Press timestamps (uptime ms) flow ISR -> UI thread. */
K_MSGQ_DEFINE(press_q, sizeof(uint32_t), 8, 4);

static struct gpio_callback btn_cb;

static void button_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    static uint32_t last_ms;
    uint32_t now = k_uptime_get_32();

    /* Only count the active (pressed) edge, debounced. */
    if (gpio_pin_get_dt(&button) != 1) {
        return;
    }
    if ((now - last_ms) < DEBOUNCE_MS) {
        return;
    }
    last_ms = now;

    k_msgq_put(&press_q, &now, K_NO_WAIT);   /* drop if full */
}

/* UI state. BLINK handling/LED come in Task 8; here we track the flag only. */
static bool blinking;

#define UI_PRIO    7
#define UI_STACK   1024

K_THREAD_STACK_DEFINE(ui_stack, UI_STACK);
static struct k_thread ui_tid;

static void ui_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    uint32_t t;

    while (1) {
        /* Wait for the first press of a gesture. */
        if (k_msgq_get(&press_q, &t, K_FOREVER) != 0) {
            continue;
        }

        if (blinking) {
            /* Any press stops blinking; consume it (no battery display). */
            blinking = false;
            ble_log_send("UI: blink stop\n");
            continue;
        }

        /* Wait up to DOUBLE_MS for a second press. */
        if (k_msgq_get(&press_q, &t, K_MSEC(DOUBLE_MS)) == 0) {
            blinking = true;
            ble_log_send("UI: double\n");
        } else {
            ble_log_send("UI: single\n");
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
