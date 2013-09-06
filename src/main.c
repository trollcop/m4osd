#include "board.h"

void lolInit(void)
{
}

void systemInit(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef spi;

    SystemCoreClockUpdate();

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);

    // Configure FPU
    FPU->FPCCR &= ~FPU_FPCCR_ASPEN_Msk;     // turn off FP context save
    FPU->FPCCR &= ~FPU_FPCCR_LSPEN_Msk;     // turn off lazy save

    // Status LED PB0, PB1
    digitalHi(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);

    // SPI2+SPI3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 | RCC_APB1Periph_SPI3, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5); // SPI2_SCK PB13
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5); // SPI2_MISO PB14
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_6); // SPI3_SCK PB3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_6); // SPI3_MISO PB4
    gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOB, &gpio);
}

OS_STK taskA_stk[128];
OS_STK taskB_stk[128];

OS_TCID timer50Hz;
OS_FlagID timer50HzFlag;
OS_STK task50Hzstk[128];

void timer50HzCallback(void)
{
    // set flag
    CoSetFlag(timer50HzFlag);
}

void task50Hz(void *pdata)
{
    static float p = 1;
    while (1) {
        CoWaitForSingleFlag(timer50HzFlag, 0);
        digitalToggle(GPIOB, GPIO_Pin_0);
        p *= 1.1f;
    }
}

void taskA(void* pdata)
{
    unsigned int led_num;
    static float p = 1;
    static float s = 0;

    for (;;) {
        led_num++;
        CoTickDelay(500);
        s = sinf(p++);
        digitalToggle(GPIOB, GPIO_Pin_1);
    }
}

void taskB(void* pdata)
{
    unsigned int led_num;

    for (;;) {
        led_num++;
        CoTickDelay(250);
        // LED0_TOGGLE;
    }
}

int main(void)
{
    lolInit();
    systemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    CoInitOS();
    timer50HzFlag = CoCreateFlag(1, 0);
    CoCreateTask(taskA, 0, 10, &taskA_stk[128 - 1], 128);
    CoCreateTask(task50Hz, 0, 15, &task50Hzstk[128 - 1], 128);
    timer50Hz = CoCreateTmr(TMR_TYPE_PERIODIC, 100, 100, timer50HzCallback);
    CoStartTmr(timer50Hz);

    CoStartOS();
    return 0;
}
