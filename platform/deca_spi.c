/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions — Zephyr RTOS port for nRF52833 tag
 *
 * SPI1 peripheral, 32 MHz max, CS on P0.11 (GPIO_ACTIVE_LOW, managed by driver).
 *
 * Copyright 2015 - 2021 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 */

#include "deca_spi.h"
#include <zephyr/kernel.h>

/*
 * nRF52833 SPIM1 supports discrete frequencies: 4 / 8 / 16 / 32 MHz.
 * 32 MHz is the maximum and is used for normal DW3000 operation.
 * 4 MHz is required during DW3000 initialisation (device wake-up).
 */
#define DW3000_SPI_SLOW_HZ   4000000U
#define DW3000_SPI_FAST_HZ  32000000U

static const struct device *const spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));

/*
 * CS is described in the board DTS as:
 *   cs-gpios = <&gpio0 11 GPIO_ACTIVE_LOW>;
 * The Zephyr SPI driver asserts/deasserts CS automatically around each transfer.
 */
static const struct spi_config spi_cfg_slow = {
    .frequency = DW3000_SPI_SLOW_HZ,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .cs = {
        .gpio  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi1), cs_gpios, 0),
        .delay = 0U,
    },
};

static const struct spi_config spi_cfg_fast = {
    .frequency = DW3000_SPI_FAST_HZ,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .cs = {
        .gpio  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi1), cs_gpios, 0),
        .delay = 0U,
    },
};

static const struct spi_config *current_cfg = &spi_cfg_slow;

static K_MUTEX_DEFINE(spi_mutex);

/*
 * Receive discard buffer for the header phase of a read transaction.
 * Sized to the maximum header length; not accessed outside this file.
 * Must be static (nRF SPIM uses EasyDMA — stack memory is not safe).
 */
static uint8_t rx_dummy[DECA_MAX_SPI_HEADER_LENGTH];

/* ---- Init / rate selection ----------------------------------------------- */

void dwm3001c_spi_init(void)
{
    __ASSERT(device_is_ready(spi_dev), "SPI1 device not ready");
    current_cfg = &spi_cfg_slow;
}

void port_set_dw_ic_spi_slowrate(void)
{
    current_cfg = &spi_cfg_slow;
}

void port_set_dw_ic_spi_fastrate(void)
{
    current_cfg = &spi_cfg_fast;
}

/* ---- SPI transfers ------------------------------------------------------- */

/*
 * Write header + body in a single CS-asserted transaction.
 * Zephyr sends the two buffers back-to-back without deassserting CS.
 */
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint16_t bodyLength, const uint8_t *bodyBuffer)
{
    int ret;
    struct spi_buf tx_bufs[] = {
        { .buf = (void *)headerBuffer, .len = headerLength },
        { .buf = (void *)bodyBuffer,   .len = bodyLength   },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count   = ARRAY_SIZE(tx_bufs),
    };

    k_mutex_lock(&spi_mutex, K_FOREVER);
    ret = spi_write(spi_dev, current_cfg, &tx);
    k_mutex_unlock(&spi_mutex);

    return ret;
}

/*
 * Write header then read data in a single CS-asserted transaction.
 * During the header phase the received bytes are discarded into rx_dummy.
 * During the data phase TX sends zeros (buf = NULL) while RX captures readBuffer.
 */
int readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                uint16_t readLength, uint8_t *readBuffer)
{
    int ret;

    /* rx_dummy is DECA_MAX_SPI_HEADER_LENGTH (3) bytes and is the DMA
     * destination for the header phase. A longer header would have SPIM's
     * EasyDMA write past it into whatever BSS follows -- silent corruption of
     * an unrelated variable, surfacing later as a wild pointer somewhere with
     * no connection to SPI. Refuse the transfer instead. */
    if (headerLength > sizeof(rx_dummy)) {
        return -EINVAL;
    }

    struct spi_buf tx_bufs[] = {
        { .buf = headerBuffer, .len = headerLength },
        { .buf = NULL,         .len = readLength   },
    };
    struct spi_buf rx_bufs[] = {
        { .buf = rx_dummy,   .len = headerLength },
        { .buf = readBuffer, .len = readLength   },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count   = ARRAY_SIZE(tx_bufs),
    };
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count   = ARRAY_SIZE(rx_bufs),
    };

    k_mutex_lock(&spi_mutex, K_FOREVER);
    ret = spi_transceive(spi_dev, current_cfg, &tx, &rx);
    k_mutex_unlock(&spi_mutex);

    return ret;
}

int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer,
                      uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8)
{
#ifdef DWT_ENABLE_CRC
    int ret;
    struct spi_buf tx_bufs[] = {
        { .buf = (void *)headerBuffer, .len = headerLength },
        { .buf = (void *)bodyBuffer,   .len = bodyLength   },
        { .buf = &crc8,                .len = 1U           },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count   = ARRAY_SIZE(tx_bufs),
    };

    k_mutex_lock(&spi_mutex, K_FOREVER);
    ret = spi_write(spi_dev, current_cfg, &tx);
    k_mutex_unlock(&spi_mutex);

    return ret;
#else
    return 0;
#endif
}
