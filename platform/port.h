/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @brief   HW specific definitions and functions for portability
 *          Zephyr RTOS port for DWM3001CDK (nRF52833)
 *
 * Copyright 2015 - 2021 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>

/*
 * DW3000 GPIO pin numbers on nRF52833 tag.
 * P0.x → x, P1.x → 32+x (nRF SDK-compatible flat numbering).
 * SPI1 pins (SCK/MOSI/MISO/CS) are managed by the Zephyr SPI driver.
 */
#define DW3000_IRQ_Pin   30   /* P0.30 */
#define DW3000_RST_Pin   31   /* P0.31 */
#define DW3000_WUP_Pin   15   /* P0.15 */
#define DW3000_CS_Pin    11   /* P0.11 */
#define DW3000_CLK_Pin    4   /* P0.04 (SPI1 SCK) */
#define DW3000_MOSI_Pin  41   /* P1.09 (SPI1 MOSI) = 32 + 9 */
#define DW3000_MISO_Pin   5   /* P0.05 (SPI1 MISO) */

/* DW IC IRQ handler type. */
typedef void (*port_dwic_isr_t)(void);

/* UART console interface (backed by Zephyr printk). */
void        uart_init(void);
void        uart_send(const char *msg);
bool        uart_line_available(void);
const char *uart_get_line(void);
void        uart_clear_line(void);

/* @fn    Sleep — sleep delay in ms */
void Sleep(uint32_t x);

int peripherals_init(void);

/* @fn    gpio_init — assert gpio0 ready; configure RST and WUP pins */
void gpio_init(void);

void reset_DWIC(void);
void wakeup_device_with_io(void);
void make_very_short_wakeup_io(void);
void process_deca_irq(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void dw_irq_init(void);
void port_set_dwic_isr(port_dwic_isr_t dwic_isr);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
