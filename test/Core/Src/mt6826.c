/**
  ******************************************************************************
  * @file    mt6826.c
  * @brief   MT6826S magnetic encoder driver (SPI-DMA based)
  ******************************************************************************
  */
#include "mt6826.h"

/* ======================================================================== */
void MT6826_Init(MT6826_HandleTypeDef *hmt, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *nss_port, uint16_t nss_pin)
{
    hmt->hspi      = hspi;
    hmt->nss_port  = nss_port;
    hmt->nss_pin   = nss_pin;
    hmt->dma_done  = 1u;       /* mark idle so first read can start */
    hmt->read_pending = 0u;
    hmt->active_sample_idx = 0u;
    hmt->sample_buf[0].angle_raw = 0u;
    hmt->sample_buf[0].status = 0u;
    hmt->sample_buf[0].crc = 0u;
    hmt->sample_buf[1] = hmt->sample_buf[0];

    /* NSS idle high */
    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);

    /* Build continuous-read command frame (constant, initialised once):
     *   Byte0 : [CMD 1010][ADDR_H 0x0]  = 0xA0
     *   Byte1 : ADDR_L = 0x03  (start at ANGLE_H register)
     *   Byte2~5: dummy bytes – clock out ANGLE_H / ANGLE_L / STATUS / CRC */
    hmt->tx_buf[0] = (uint8_t)(MT6826S_CMD_CONT_READ << 4u);   /* 0xA0 */
    hmt->tx_buf[1] = 0x03u;
    hmt->tx_buf[2] = 0x00u;
    hmt->tx_buf[3] = 0x00u;
    hmt->tx_buf[4] = 0x00u;
    hmt->tx_buf[5] = 0x00u;
}

/* ======================================================================== */
uint8_t MT6826_ReadReg(MT6826_HandleTypeDef *hmt, uint16_t addr)
{
    uint8_t tx[3], rx[3] = {0u, 0u, 0u};
    tx[0] = (uint8_t)((MT6826S_CMD_READ << 4u) | ((addr >> 8u) & 0x0Fu));
    tx[1] = (uint8_t)(addr & 0xFFu);
    tx[2] = 0x00u;

    HAL_GPIO_WritePin(hmt->nss_port, hmt->nss_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hmt->hspi, tx, rx, 3u, 10u);
    HAL_GPIO_WritePin(hmt->nss_port, hmt->nss_pin, GPIO_PIN_SET);

    return rx[2];
}

/* ======================================================================== */
void MT6826_WriteReg(MT6826_HandleTypeDef *hmt, uint16_t addr, uint8_t val)
{
    uint8_t tx[3], rx[3] = {0u, 0u, 0u};
    tx[0] = (uint8_t)((MT6826S_CMD_WRITE << 4u) | ((addr >> 8u) & 0x0Fu));
    tx[1] = (uint8_t)(addr & 0xFFu);
    tx[2] = val;

    HAL_GPIO_WritePin(hmt->nss_port, hmt->nss_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hmt->hspi, tx, rx, 3u, 10u);
    HAL_GPIO_WritePin(hmt->nss_port, hmt->nss_pin, GPIO_PIN_SET);
}

/* ======================================================================== */
void MT6826_StartDMARead(MT6826_HandleTypeDef *hmt)
{
    hmt->read_pending = 0u;
    hmt->dma_done = 0u;
    HAL_GPIO_WritePin(hmt->nss_port, hmt->nss_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hmt->hspi, hmt->tx_buf,
                                (uint8_t *)hmt->rx_buf, 6u);
}

/* ======================================================================== */
void MT6826_DMACompleteCallback(MT6826_HandleTypeDef *hmt)
{
    uint8_t write_idx;

    /* Release chip-select */
    HAL_GPIO_WritePin(hmt->nss_port, hmt->nss_pin, GPIO_PIN_SET);

    /* Parse received frame:
     *   rx_buf[0..1] : invalid (command/address phase, MISO tri-state)
     *   rx_buf[2]    : ANGLE[14:7]
     *   rx_buf[3]    : ANGLE[6:0] in bit[7:1], bit0 fixed 0
     *   rx_buf[4]    : STATUS[2:0]
     *   rx_buf[5]    : CRC[7:0]   */
    write_idx = (uint8_t)(hmt->active_sample_idx ^ 1u);
    hmt->sample_buf[write_idx].angle_raw =
        ((uint16_t)hmt->rx_buf[2] << 7u) | (hmt->rx_buf[3] >> 1u);
    hmt->sample_buf[write_idx].status = hmt->rx_buf[4];
    hmt->sample_buf[write_idx].crc = hmt->rx_buf[5];
    __DMB();
    hmt->active_sample_idx = write_idx;
    __DMB();
    hmt->dma_done  = 1u;
}

/* ======================================================================== */
uint16_t MT6826_GetAngleRaw(const MT6826_HandleTypeDef *hmt)
{
    return hmt->sample_buf[hmt->active_sample_idx].angle_raw;
}

/* ======================================================================== */
void MT6826_GetLatestSample(const MT6826_HandleTypeDef *hmt,
                            MT6826_SampleTypeDef *sample)
{
    uint8_t active_idx;

    if (sample == NULL) return;

    active_idx = hmt->active_sample_idx;
    *sample = hmt->sample_buf[active_idx];
}
