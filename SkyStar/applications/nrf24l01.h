/*
 * nrf24l01.h
 * NRF24L01+ 底层驱动 (HAL SPI2)
 * 配对模式: 增强 ShockBurst (DPL + ACK Payload), 与遥控器 MODEL_TX2 兼容
 */
#ifndef APPLICATIONS_NRF24L01_H_
#define APPLICATIONS_NRF24L01_H_

#include "main.h"
#include <stdint.h>

/* ---- 引脚 (参考 cubemx.ioc) ---- */
#define NRF_CE_PORT     GPIOA
#define NRF_CE_PIN      GPIO_PIN_8
#define NRF_CSN_PORT    GPIOA
#define NRF_CSN_PIN     GPIO_PIN_15

#define NRF_CE_LOW()    HAL_GPIO_WritePin(NRF_CE_PORT,  NRF_CE_PIN,  GPIO_PIN_RESET)
#define NRF_CE_HIGH()   HAL_GPIO_WritePin(NRF_CE_PORT,  NRF_CE_PIN,  GPIO_PIN_SET)
#define NRF_CSN_LOW()   HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET)
#define NRF_CSN_HIGH()  HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET)

/* ---- SPI 命令字 ---- */
#define SPI_READ_REG    0x00
#define SPI_WRITE_REG   0x20
#define RD_RX_PLOAD     0x61
#define R_RX_PL_WID     0x60
#define W_ACK_PAYLOAD   0xA8
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define NRF_NOP         0xFF

/* ---- 寄存器 ---- */
#define NCONFIG         0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define RX_ADDR_P0      0x0A
#define TX_ADDR         0x10
#define FIFO_STATUS     0x17
#define DYNPD           0x1C
#define FEATURE         0x1D

/* ---- STATUS 位 ---- */
#define STA_RX_DR       0x40
#define STA_TX_DS       0x20
#define STA_MAX_RT      0x10

/* ---- 配置 ---- */
#define ADDR_WIDTH      5

/* ---- 全局 ---- */
extern uint8_t  NRF24L01_RXDATA[32];
extern volatile uint8_t nrf_data_ready;   /* 1=有新包待解析, 0=已消费 */

/* ---- API ---- */
void    NRF24L01_Configuration(void);
uint8_t NRF24L01_Check(void);
void    ANO_NRF_Init(uint8_t ch);
void    ANO_NRF_SetChannel(uint8_t ch);
void    ANO_NRF_Check_Event(void);

uint8_t NRF24L01_Read_Reg(uint8_t reg);

#endif
