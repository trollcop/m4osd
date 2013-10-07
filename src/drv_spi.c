#include "board.h"

typedef struct spiSlave_t {
    GPIO_TypeDef *gpio;
    uint16_t pin;
} spiSlave_t;

static const spiSlave_t slaves[] = {
    { GPIOC, GPIO_Pin_13 }, // PC13 Flash Chip Select
    { GPIOB, GPIO_Pin_2 }   //  PB2 VDAC Chip Select
};

void spiInit(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef spi;
    int i;

    // SPI1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5); // SPI1_SCK PA5
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5); // SPI1_MISO PA6
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5); // SPI1_MOSI PA7
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &gpio);

    // Chip Selects PC13 (flash), PB2 (levels DAC)
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    for (i = 0; i < 2; i++) {
        digitalHi(slaves[i].gpio, slaves[i].pin);
        gpio.GPIO_Pin = slaves[i].pin;
        GPIO_Init(slaves[i].gpio, &gpio);
    }

    // SPI1 configuration
    SPI_I2S_DeInit(SPI1);
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &spi);
    SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
    SPI_Cmd(SPI1, ENABLE);
}

void spiSelect(int client)
{
    digitalLo(slaves[client].gpio, slaves[client].pin);
}

void spiDeselect(int client)
{
    digitalHi(slaves[client].gpio, slaves[client].pin);
}

uint8_t spiTransferByte(uint8_t in)
{
    uint8_t rx;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_SendData8(SPI1, in);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    rx = SPI_ReceiveData8(SPI1);
    return rx;
}

int spiTransfer(uint8_t *out, uint8_t *in, int len)
{
    uint8_t b;
    while (len--) {
        b = in ? *(in++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
        SPI_SendData8(SPI1, b);
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
        b = SPI_ReceiveData8(SPI1);
        if (out)
            *(out++) = b;
    }

    return len;
}

/*
 * DAC082S085 support
 *
 * The DAC requires CPOL_Low to update voltage, so SPI peripheral needs to be reconfigured
 * Default config for the SPI1 driver is CPOL_High, to access SPI Flash. DAC access toggles
 * it to CPOL_Low then sets it back to CPOL_High after transfer.
 *
 * DAC Data format (16bits, MSB first)
 * A1 A0 OP1 OP0 D7 D6 D5 D4 | D3 D2 D1 D0 XX XX XX XX
 *
 * DAC Output voltage 0..255 = 0..3300mV
 */
void vdacVoltage(uint8_t dac, int voltage)
{
    uint8_t buf[2];
    uint8_t value = (voltage * 255 / 3300);
    buf[0] = (dac << 6) | (0x1 << 4) | ((value >> 4) & 0xF);
    buf[1] = ((value & 0xF) << 4);

    SPI1->CR1 &= ~SPI_CR1_SPE; // Disable SPI
    SPI1->CR1 &= ~SPI_CR1_CPOL; // SPI_CPOL_Low
    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI

    spiSelect(SPI_VDAC);
    spiTransfer(NULL, buf, 2);
    spiDeselect(SPI_VDAC);

    SPI1->CR1 &= ~SPI_CR1_SPE; // Disable SPI
    SPI1->CR1 |= SPI_CR1_CPOL; // SPI_CPOL_High
    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}
