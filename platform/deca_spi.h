/*! ----------------------------------------------------------------------------
 * @file    deca_spi.h
 * @brief   SPI access functions — Zephyr RTOS port for DWM3001CDK (nRF52833)
 *
 * Copyright 2015 - 2021 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 */

#ifndef _DECA_SPI_H_
#define _DECA_SPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <zephyr/drivers/spi.h>
#include <deca_types.h>

#define DECA_MAX_SPI_HEADER_LENGTH (3)
#define DATALEN1                   200

/* @fn    dwm3001c_spi_init
 * @brief Assert SPI3 device is ready and select slow rate.
 */
void dwm3001c_spi_init(void);

/* @fn    port_set_dw_ic_spi_slowrate — 4 MHz (used during DW3000 init) */
void port_set_dw_ic_spi_slowrate(void);

/* @fn    port_set_dw_ic_spi_fastrate — 32 MHz (normal operation) */
void port_set_dw_ic_spi_fastrate(void);

int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer,
                      uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8);

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint16_t bodyLength, const uint8_t *bodyBuffer);

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                uint16_t readLength, uint8_t *readBuffer);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_SPI_H_ */
