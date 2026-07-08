/*
 * nrf24l01.c
 * NRF24L01+ 底层 (HAL SPI2): 增强 ShockBurst RX
 */
#include "nrf24l01.h"

static const uint8_t ADDR[ADDR_WIDTH] = {0xAA, 0xBB, 0xCC, 0x00, 0x01};

uint8_t NRF24L01_RXDATA[32];
volatile uint8_t nrf_data_ready = 0;

static SPI_HandleTypeDef hspi2_nrf;

static uint8_t spi_rw(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2_nrf, &tx, &rx, 1, 100);
    return rx;
}

uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t v;
    NRF_CSN_LOW();
    spi_rw(reg);
    v = spi_rw(NRF_NOP);
    NRF_CSN_HIGH();
    return v;
}

static void write_reg(uint8_t reg, uint8_t val)
{
    NRF_CSN_LOW();
    spi_rw(reg);
    spi_rw(val);
    NRF_CSN_HIGH();
}

static void write_buf(uint8_t reg, const uint8_t *p, uint8_t len)
{
    NRF_CSN_LOW();
    spi_rw(reg);
    while (len--) spi_rw(*p++);
    NRF_CSN_HIGH();
}

static void read_buf(uint8_t reg, uint8_t *p, uint8_t len)
{
    NRF_CSN_LOW();
    spi_rw(reg);
    while (len--) *p++ = spi_rw(NRF_NOP);
    NRF_CSN_HIGH();
}

void NRF24L01_Configuration(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    /* SPI2: PB13=SCK, PC2=MISO, PC3=MOSI */
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    g.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOC, &g);

    /* CE / CSN 推挽 */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Alternate = 0;
    g.Pin = NRF_CE_PIN;
    HAL_GPIO_Init(NRF_CE_PORT, &g);
    g.Pin = NRF_CSN_PIN;
    HAL_GPIO_Init(NRF_CSN_PORT, &g);

    hspi2_nrf.Instance               = SPI2;
    hspi2_nrf.Init.Mode              = SPI_MODE_MASTER;
    hspi2_nrf.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2_nrf.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2_nrf.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2_nrf.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2_nrf.Init.NSS               = SPI_NSS_SOFT;
    hspi2_nrf.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi2_nrf.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2_nrf.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2_nrf.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2_nrf.Init.CRCPolynomial     = 10;
    HAL_SPI_Init(&hspi2_nrf);

    NRF_CE_LOW();
    NRF_CSN_HIGH();
}

uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t rb[5]  = {0};
    uint8_t i;
    write_buf(SPI_WRITE_REG + TX_ADDR, buf, 5);
    read_buf(TX_ADDR, rb, 5);
    for (i = 0; i < 5; i++) if (rb[i] != 0xA5) return 1;
    return 0;
}

void ANO_NRF_Init(uint8_t ch)
{
    NRF_CE_LOW();
    write_buf(SPI_WRITE_REG + RX_ADDR_P0, ADDR, ADDR_WIDTH);
    write_buf(SPI_WRITE_REG + TX_ADDR,    ADDR, ADDR_WIDTH);
    write_reg(SPI_WRITE_REG + EN_AA,       0x01);
    write_reg(SPI_WRITE_REG + EN_RXADDR,   0x01);
    write_reg(SPI_WRITE_REG + SETUP_RETR,  0x1A);
    write_reg(SPI_WRITE_REG + RF_CH,       ch);
    write_reg(SPI_WRITE_REG + RF_SETUP,    0x0F);
    write_reg(FLUSH_TX, 0xFF);
    write_reg(FLUSH_RX, 0xFF);
    write_reg(SPI_WRITE_REG + NCONFIG, 0x0F);    /* PWR_UP+CRC16+PRIM_RX */

    /* 激活 DPL + ACK_PAY */
    NRF_CSN_LOW();
    spi_rw(0x50);
    spi_rw(0x73);
    NRF_CSN_HIGH();
    write_reg(SPI_WRITE_REG + DYNPD,   0x01);
    write_reg(SPI_WRITE_REG + FEATURE, 0x06);

    NRF_CE_HIGH();
}

void ANO_NRF_SetChannel(uint8_t ch)
{
    NRF_CE_LOW();
    write_reg(SPI_WRITE_REG + RF_CH, ch);
    write_reg(FLUSH_RX, 0xFF);
    write_reg(FLUSH_TX, 0xFF);
    write_reg(SPI_WRITE_REG + STATUS, 0x70);
    nrf_data_ready = 0;
    NRF_CE_HIGH();
}

/*
 * 收到一帧后必须装载 ACK Payload, 遥控器据此判定"找到飞机"并锁频
 * payload 内容随便, 这里发一个最小的 AA AA 帧 (与小车工程一致)
 */
void ANO_NRF_Check_Event(void)
{
    uint8_t sta = NRF24L01_Read_Reg(STATUS);

    if (sta & STA_RX_DR) {
        uint8_t len = NRF24L01_Read_Reg(R_RX_PL_WID);
        if (len > 0 && len <= 32) {
            read_buf(RD_RX_PLOAD, NRF24L01_RXDATA, len);
            nrf_data_ready = 1;
        } else {
            write_reg(FLUSH_RX, 0xFF);
        }
        if (!(NRF24L01_Read_Reg(FIFO_STATUS) & 0x20)) {
            static const uint8_t ack[8] = {0xAA, 0xAA, 0x05, 0x03,
                                           0x00, 0x00, 0x00, 0xB6};
            NRF_CSN_LOW();
            spi_rw(W_ACK_PAYLOAD);
            for (uint8_t i = 0; i < 8; i++) spi_rw(ack[i]);
            NRF_CSN_HIGH();
        }
    }
    if (sta & STA_MAX_RT) write_reg(FLUSH_TX, 0xFF);
    write_reg(SPI_WRITE_REG + STATUS, sta);
}
