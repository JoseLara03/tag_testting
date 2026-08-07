/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability
 *          Zephyr RTOS port for DWM3001CDK (nRF52833)
 *
 * Copyright 2016 - 2021 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 */

#include "port.h"

static const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

/* Every pin driven directly below goes through gpio0, which covers P0.00-P0.31.
 * The nRF driver does not reject an out-of-range pin, it masks it:
 * NRF_PIN_PORT_TO_PIN_NUMBER(pin, port) = ((pin) & 0x1F) | ((port) << 5), and
 * Zephyr's own range check is an __ASSERT that compiles out. So a P1.x value in
 * this file's flat numbering silently lands on a different P0 pin -- RST was 37
 * (P1.05 in that scheme) and aliased onto P0.05, the SPI1 MISO line, putting the
 * nRF's output driver against the DW3000's. Fail the build instead. */
BUILD_ASSERT(DW3000_RST_Pin < 32, "DW3000_RST_Pin is not on gpio0");
BUILD_ASSERT(DW3000_IRQ_Pin < 32, "DW3000_IRQ_Pin is not on gpio0");
BUILD_ASSERT(DW3000_WUP_Pin < 32, "DW3000_WUP_Pin is not on gpio0");
BUILD_ASSERT(DW3000_CS_Pin  < 32, "DW3000_CS_Pin is not on gpio0");

/* ---- UART (Zephyr printk-backed) ---------------------------------------- */

#define UART_LINE_LEN 64

static char    s_line_buf[UART_LINE_LEN];
static uint8_t s_line_pos   = 0;
static bool    s_line_ready = false;

void uart_init(void) { }

void uart_send(const char *msg)
{
    printk("%s\n", msg);
}

/* RTT input is not available in the Zephyr port; always returns false. */
bool uart_line_available(void)
{
    return s_line_ready;
}

const char *uart_get_line(void) { return s_line_buf; }

void uart_clear_line(void)
{
    s_line_pos   = 0;
    s_line_ready = false;
}

/* ---- Time ---------------------------------------------------------------- */

void Sleep(uint32_t x)
{
    k_msleep(x);
}

/* ---- Peripherals --------------------------------------------------------- */

int peripherals_init(void)
{
    return 0;
}

/* ---- GPIO ---------------------------------------------------------------- */

void gpio_init(void)
{
    __ASSERT(device_is_ready(gpio0), "gpio0 device not ready");

    /* RST: no more level-shifting transistor now that DW3000 and nRF share 3.3V.
     * RSTn is open-drain on the DW3000 side with its own pull-up, so the nRF pin
     * must idle as a high-impedance input; driving it high would fight the DW3000
     * if it (or anything else) pulls the line low. reset_DWIC() switches it to an
     * output only for the duration of the reset pulse. */
    gpio_pin_configure(gpio0, DW3000_RST_Pin, GPIO_INPUT);

    /* WUP: external pull-down holds it low at rest; drive HIGH to wake DW3000. */
    gpio_pin_configure(gpio0, DW3000_WUP_Pin, GPIO_OUTPUT_INACTIVE);
}

/* ---- DW3000 IRQ ---------------------------------------------------------- */

static struct gpio_callback dw3000_irq_cb;
static port_dwic_isr_t      port_dwic_isr = NULL;

static void deca_irq_handler(const struct device *dev,
                              struct gpio_callback *cb,
                              uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    process_deca_irq();
}

void dw_irq_init(void)
{
    /* IRQ: external pull-down; DW3000 drives high to signal. No internal pull needed. */
    gpio_pin_configure(gpio0, DW3000_IRQ_Pin, GPIO_INPUT);
    gpio_init_callback(&dw3000_irq_cb, deca_irq_handler, BIT(DW3000_IRQ_Pin));
    gpio_add_callback(gpio0, &dw3000_irq_cb);
    gpio_pin_interrupt_configure(gpio0, DW3000_IRQ_Pin, GPIO_INT_EDGE_RISING);
}

/* ---- DW3000 control ------------------------------------------------------ */

void reset_DWIC(void)
{
    /* Assert reset. OUTPUT_INACTIVE configures the pin and drives it low in one
     * step; OUTPUT_ACTIVE would drive it high first, and an open-drain RSTn must
     * never be driven high. */
    gpio_pin_configure(gpio0, DW3000_RST_Pin, GPIO_OUTPUT_INACTIVE);
    k_msleep(2);

    /* Release: go back to high-impedance input and let the DW3000's own
     * pull-up bring RSTn high. Do NOT drive the pin high. */
    gpio_pin_configure(gpio0, DW3000_RST_Pin, GPIO_INPUT);
    k_msleep(2);
}

void wakeup_device_with_io(void)
{
    /* DW3000 requires the WAKEUP pin held high for at least 500 µs to wake from
     * SLEEP/DEEPSLEEP (DW3 SDK examples, NOTE 5).  600 µs gives margin. */
    gpio_pin_set(gpio0, DW3000_WUP_Pin, 1);
    k_busy_wait(600);
    gpio_pin_set(gpio0, DW3000_WUP_Pin, 0);
}

void make_very_short_wakeup_io(void)
{
    uint8_t cnt;

    gpio_pin_set(gpio0, DW3000_WUP_Pin, 1);
    for (cnt = 0; cnt < 10; cnt++) {
        arch_nop();
    }
    gpio_pin_set(gpio0, DW3000_WUP_Pin, 0);
}

/* ---- IRQ routing --------------------------------------------------------- */

void process_deca_irq(void)
{
    while (port_CheckEXT_IRQ() != 0) {
        if (port_dwic_isr) {
            port_dwic_isr();
        }
    }
}

static volatile bool dw_irq_enabled = false;

void port_DisableEXT_IRQ(void)
{
    gpio_pin_interrupt_configure(gpio0, DW3000_IRQ_Pin, GPIO_INT_DISABLE);
    dw_irq_enabled = false;
}

void port_EnableEXT_IRQ(void)
{
    gpio_pin_interrupt_configure(gpio0, DW3000_IRQ_Pin, GPIO_INT_EDGE_RISING);
    dw_irq_enabled = true;
}

uint32_t port_GetEXT_IRQStatus(void)
{
    return dw_irq_enabled ? 1u : 0u;
}

uint32_t port_CheckEXT_IRQ(void)
{
    return (uint32_t)gpio_pin_get_raw(gpio0, DW3000_IRQ_Pin);
}

void port_set_dwic_isr(port_dwic_isr_t dwic_isr)
{
    /* Read pin state before disabling so we know whether to re-enable. */
    uint32_t en = port_GetEXT_IRQStatus();

    port_DisableEXT_IRQ();
    port_dwic_isr = dwic_isr;

    if (en) {
        port_EnableEXT_IRQ();
    }
}
