#include "board.h"
#include "osdcore.h"

static uint32_t zero = 0;
osdData_t osdData;
extern const unsigned char afrologo[];

static void rageMemsetInit(void)
{
    DMA_InitTypeDef dma;
    DMA_StructInit(&dma);

    dma.DMA_PeripheralBaseAddr = (uint32_t)&zero;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Enable;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_BufferSize = sizeof(osdData.OSD_RAM) / 4;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_MemoryBaseAddr = (uint32_t)osdData.OSD_RAM;
    DMA_DeInit(MEMSET_DMA);
    DMA_Init(MEMSET_DMA, &dma);
}

static __inline void rageMemset(void)
{
    MEMSET_DMA->CCR &= (uint16_t)(~DMA_CCR_EN);
    MEMSET_DMA->CMAR = (uint32_t)osdData.OSD_RAM;
    MEMSET_DMA->CNDTR = sizeof(osdData.OSD_RAM);
    MEMSET_DMA->CCR |= DMA_CCR_EN;
}

void osdInit(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;
    DMA_InitTypeDef dma;
    SPI_InitTypeDef spi;
    TIM_OCInitTypeDef timoc;

    // turn on peripherals. we're using DMA1, DMA2, SPI2, SPI3, TIM1, TIM8 here
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 | RCC_APB1Periph_SPI3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8, ENABLE);

    // dma-based memory clear
    rageMemsetInit();

    // OSD SPI2 + SPI3 GPIO
    GPIO_StructInit(&gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5); // SPI2_SCK PB13
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5); // SPI2_MISO PB14
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_6); // SPI3_SCK PB3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_6); // SPI3_MISO PB4
    gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOB, &gpio);

    // White SPI
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_1Line_Tx;
    spi.SPI_Mode = SPI_Mode_Slave;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_FirstBit = 0;
    spi.SPI_CRCPolynomial = 0;
    SPI_Init(WHITE_SPI, &spi);
    SPI_RxFIFOThresholdConfig(WHITE_SPI, SPI_RxFIFOThreshold_QF);
    SPI_Cmd(WHITE_SPI, ENABLE);

    // White DMA DMA1_Channel5
    DMA_StructInit(&dma);
    DMA_DeInit(WHITE_DMA);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&WHITE_SPI->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)osdData.OSD_LINE;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = OSD_HRES;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(WHITE_DMA, &dma);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);      // enable

    // OSD DMA for SPI setup
    DMA_Cmd(WHITE_DMA, ENABLE);
    SPI_I2S_DMACmd(WHITE_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

    // Black SPI
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_1Line_Tx;
    spi.SPI_Mode = SPI_Mode_Slave;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_FirstBit = 0;
    spi.SPI_CRCPolynomial = 0;
    SPI_Init(BLACK_SPI, &spi);
    SPI_RxFIFOThresholdConfig(BLACK_SPI, SPI_RxFIFOThreshold_QF);
    SPI_Cmd(BLACK_SPI, ENABLE);


#if 0


    // BW SPI2 Pins
    gpio.GPIO_Pin = GPIO_Pin_14; // SPI2_MISO
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15; // SPI2_SCK SPI2_MOSI
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);


    // OSDBW DMA DMA1_Channel5
    DMA_StructInit(&dma);
    DMA_DeInit(OSDBW_DMA);
    dma.DMA_PeripheralBaseAddr = (uint32_t) &OSDBW_SPI->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t) osdData.OSD_RAM;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = OSD_HRES;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh; 
    //dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(OSDBW_DMA, &dma);

    // OSDBW DMA for SPI
    DMA_Cmd(OSDBW_DMA, ENABLE);
    SPI_I2S_DMACmd(OSDBW_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
#endif

    // HSYNC
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6); // TIM1_CH1 PA8
    gpio.GPIO_Pin = GPIO_Pin_8; // TIM1_CH1 HSYNC
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // VSYNC
    gpio.GPIO_Pin = GPIO_Pin_15; // TIM15_CH2 VSYNC (or GPIO)?
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);

    // OSD Timer (TIM1@72MHz)
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Period = 0xFFFF;
    tim.TIM_Prescaler = 0;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tim);

    // reset timer on each edge (HSYNC)
    TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
    TIM_ETRConfig(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 50);
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);

    // set compare value
    TIM_SetCompare1(TIM1, 80); // shift left/right osd screen. timing depended from irq handler

    nvic.NVIC_IRQChannel = TIM1_CC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_Cmd(TIM1, ENABLE);

    // SPI timing GPIO out (TIM8_CH1, PA15, AF2)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_2);
    gpio.GPIO_Pin = GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // TIM8_CH1 clk for SPI2/SPI3 in slave
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = (1 - 1);

                                   // 5  - 14.39885 ~758
                                   // 6  - 11.99904 ~631
                                   // 7  - 10.28489 ~541
                                   // 8  - 8.99928  ~473
                                   // 9  - 7.99936  ~421
    tim.TIM_Period = 10;           // 10 - 7.199424 ~378 NTSC pixels
                                   // 11 - 6.54931  ~344
                                   // 12 - 5.99952  ~315

    tim.TIM_ClockDivision = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &tim);

    TIM_OCStructInit(&timoc);
    timoc.TIM_OutputState = TIM_OutputState_Enable;
    timoc.TIM_OutputNState = TIM_OutputNState_Disable;
    timoc.TIM_OCMode = TIM_OCMode_PWM2;
    timoc.TIM_Pulse = 2;
    timoc.TIM_OCPolarity = TIM_OCPolarity_Low;
    timoc.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM8, &timoc);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    // TIM_Cmd(TIM8, ENABLE);

    // setup MEMCPY_DMA for framebuffer->line copy
    DMA_StructInit(&dma);
    DMA_DeInit(MEMCPY_DMA);
    dma.DMA_PeripheralBaseAddr = (uint32_t)osdData.OSD_LINE;
    dma.DMA_MemoryBaseAddr = (uint32_t)osdData.OSD_RAM;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = OSD_HRES;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(MEMCPY_DMA, &dma);
    //NVIC_EnableIRQ(DMA1_Channel2_IRQn);   // maybe we need irq for BW transfer

    memset((void *)osdData.OSD_LINE, 0x00, sizeof(osdData.OSD_LINE));       // last byte must be 0
    memset((void *)osdData.OSD_LINEB, 0x00, sizeof(osdData.OSD_LINEB));       // last byte must be 0
    memset((void *)osdData.OSD_RAM, 0x00, sizeof(osdData.OSD_RAM));
    memcpy((void *)osdData.OSD_RAM, afrologo, 352 / 8 * 240);

    osdData.PAL = 0;  // default is NTSC
    osdData.Height = OSD_HEIGHT_NTSC;

    osdData.osdUpdateFlag = CoCreateFlag(0, 0);
}

// WHITE_DMA (end of pixels line)
void WHITE_DMA_IRQHandler(void) 
{
    CoEnterISR();
    if (DMA1->ISR & DMA_ISR_TCIF5) {            // got end of transfer
        WHITE_DMA->CCR &= (uint16_t)(~DMA_CCR_EN);
        DMA1->IFCR |= DMA_ISR_TCIF5;
        while (WHITE_SPI->SR & SPI_SR_BSY);     // wait SPI for last bits
        TIM_Cmd(TIM8, DISABLE);                 // shut up my dear sck

        // let fill next line for video out
        MEMCPY_DMA->CCR &= (uint16_t)(~DMA_CCR_EN);
        MEMCPY_DMA->CMAR = (uint32_t)osdData.ptrOSD_RAM + OSD_HRES;     // src
        MEMCPY_DMA->CPAR = (uint32_t)osdData.OSD_LINE;                  // dst
        MEMCPY_DMA->CNDTR = OSD_HRES;
        MEMCPY_DMA->CCR |= DMA_CCR_EN;
    }
    CoExitISR();
}

// HSYNC
// PAL/SECAM have 625/50Hz and 288 active, NTSC 525 and 243 (242) active
void TIM1_CC_IRQHandler(void)
{
    int slpos, slmax;
    static int maxline = 0;
    static int inv = 0;
    int line;

    CoEnterISR();

    osdData.Height = osdData.PAL ? OSD_HEIGHT_PAL : OSD_HEIGHT_NTSC;
    slpos = osdData.PAL ? 29 /* 46 */ : 25;      // used for shift up/down area screen
    slmax = slpos + osdData.Height;

    if (TIM1->SR & TIM_SR_CC1IF) {      // capture interrupt
        TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
        if (!(GPIOB->IDR & GPIO_Pin_15) && (osdData.currentScanLine > 200)) {    // wait VSYNC
            // Note: got max 309-314 for PAL and must be 264 and more in NTSC
            if (maxline < osdData.currentScanLine)
                maxline = osdData.currentScanLine;

            osdData.maxScanLine = maxline;
            osdData.PAL = maxline > 300 ? 1 : 0;        // recheck mode
            osdData.currentScanLine = 0;
            maxline = 0;
        } else if (GPIOA->IDR & GPIO_Pin_8) {   // check HSYNC
            LED1_ON;
            osdData.currentScanLine++;

            if (osdData.currentScanLine >= slpos && osdData.currentScanLine <= slmax - 1) {
                line = osdData.currentScanLine - slpos;
                osdData.ptrOSD_RAM = (uint8_t *)&osdData.OSD_RAM[OSD_HRES * line];
                // SPI1 DMA out pixels
                WHITE_DMA->CCR &= (uint16_t)(~DMA_CCR_EN);
                WHITE_DMA->CMAR = (uint32_t)osdData.OSD_LINE;
                WHITE_DMA->CNDTR = OSD_HRES + 1;              // last byte must be zero always 
                WHITE_DMA->CCR |= DMA_CCR_EN | DMA_CCR_TCIE;
                // start video out
                TIM_Cmd(TIM8, ENABLE);
                // testing
                vdacVoltage(0, 600 - line);
            }

            // first line pre-fill, next will be filled in DMA irq
            if (osdData.currentScanLine == slpos - 1) {
                MEMCPY_DMA->CCR &= (uint16_t)(~DMA_CCR_EN);
                MEMCPY_DMA->CMAR = (uint32_t)osdData.OSD_RAM;    // src
                MEMCPY_DMA->CPAR = (uint32_t)osdData.OSD_LINE;   // dst
                MEMCPY_DMA->CNDTR = OSD_HRES;
                MEMCPY_DMA->CCR |= DMA_CCR_EN;
            }
            LED1_OFF;
#if 0
            // clear prev OSD_RAM line
            if (inv && osdData.currentScanLine >= slpos && osdData.currentScanLine <= slmax - 1) {
                line = osdData.currentScanLine - slpos;
                DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR_EN);
                DMA1_Channel1->CMAR = (uint32_t)&osdData.OSD_RAM[OSD_HRES * line]; 
                DMA1_Channel1->CNDTR = OSD_HRES;
                if (inv) 
                    DMA1_Channel1->CCR |= DMA_CCR_EN; 
            }

            // we have last line in frame?
            if (osdData.currentScanLine == slmax) {     // max line
                inv ^= 1;
                if (inv) {      // no need to redraw each frame
                    //osdClearScreen();
                    isr_SetFlag(osdData.osdUpdateFlag);   // redraw after even frame
                    } else {
                    isr_SetFlag(osdData.osdRecalcFlag);   // recalc after odd frame
               }
            }
#endif
        }
    }
    CoExitISR();
}
