/**
  ******************************************************************************
  * @file    mt6826.h
  * @brief   MT6826S magnetic encoder driver (SPI-DMA based)
  ******************************************************************************
  */
#ifndef MT6826_H
#define MT6826_H

#include "main.h"

/* --------------- MT6826S SPI command nibbles (C3~C0) ---------------------- */
#define MT6826S_CMD_READ       0x03u  /* 0011: read register              */
#define MT6826S_CMD_WRITE      0x06u  /* 0110: write register             */
#define MT6826S_CMD_CONT_READ  0x0Au  /* 1010: continuous read from addr  */

/* --------------- MT6826S register addresses ------------------------------- */
#define MT6826S_REG_USER_ID    0x001u
#define MT6826S_REG_ANGLE_H    0x003u  /* ANGLE[14:7]                     */
#define MT6826S_REG_ANGLE_L    0x004u  /* ANGLE[6:0] in bit[7:1], b0 = 0  */
#define MT6826S_REG_STATUS     0x005u  /* STATUS[2:0]                     */
#define MT6826S_REG_CRC        0x006u  /* CRC[7:0]                        */

typedef struct {
  uint16_t angle_raw;
  uint8_t  status;
  uint8_t  crc;
} MT6826_SampleTypeDef;

/* --------------- Handle -------------------------------------------------  */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *nss_port;
    uint16_t           nss_pin;
    uint8_t            tx_buf[6];          /* DMA TX frame (fixed)         */
    volatile uint8_t   rx_buf[6];          /* DMA RX frame                 */
    volatile uint8_t   dma_done;           /* 1 = transfer completed       */
  volatile uint8_t   read_pending;       /* 1 = waiting for shared SPI   */
  volatile uint8_t   active_sample_idx;  /* published ping-pong index    */
  MT6826_SampleTypeDef sample_buf[2];
} MT6826_HandleTypeDef;

/* --------------- API ----------------------------------------------------- */
void     MT6826_Init(MT6826_HandleTypeDef *hmt, SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *nss_port, uint16_t nss_pin);
uint8_t  MT6826_ReadReg(MT6826_HandleTypeDef *hmt, uint16_t addr);
void     MT6826_WriteReg(MT6826_HandleTypeDef *hmt, uint16_t addr, uint8_t val);
void     MT6826_StartDMARead(MT6826_HandleTypeDef *hmt);
void     MT6826_DMACompleteCallback(MT6826_HandleTypeDef *hmt);
uint16_t MT6826_GetAngleRaw(const MT6826_HandleTypeDef *hmt);
void     MT6826_GetLatestSample(const MT6826_HandleTypeDef *hmt,
                                MT6826_SampleTypeDef *sample);

#endif /* MT6826_H */
