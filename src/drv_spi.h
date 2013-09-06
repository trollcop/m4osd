#pragma once

void spiInit(void);
void spiSelect(int client);
void spiDeselect(int client);
uint8_t spiTransferByte(uint8_t in);
int spiTransfer(uint8_t *out, uint8_t *in, int len);

enum {
    SPI_FLASH = 0,
    SPI_VDAC = 1,
};

// @dac = DAC index, 0 = white level, 1 = black level
// @voltage = Voltage to output in mV
void vdacVoltage(uint8_t dac, int voltage);
