#include "board.h"
#include "osdcore.h"

#define BLINK_STACK_SIZE 48
OS_STK blinkStack[BLINK_STACK_SIZE];
volatile uint32_t minCycles, idleCounter, totalCycles;
uint32_t oldIdleCounter;
volatile float idlePercent;

void blinkTask(void *unused)
{
    while (1) {
        CoTickDelay(250);
        LED0_TOGGLE;
        idlePercent = 100.0f * (idleCounter - oldIdleCounter) * minCycles / totalCycles;
        oldIdleCounter = idleCounter;
        totalCycles = 0;
    }
}

void CoIdleTask(void *pdata)
{
    volatile unsigned long cycles;
    volatile unsigned int *DWT_CYCCNT = (volatile unsigned int *)0xE0001004;
    volatile unsigned int *DWT_CONTROL = (volatile unsigned int *)0xE0001000;
    volatile unsigned int *SCB_DEMCR = (volatile unsigned int *)0xE000EDFC;

    *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
    *DWT_CONTROL = *DWT_CONTROL | 1;    // enable the counter

    minCycles = 99999999;
    while (1) {
        idleCounter++;

        __nop();
        __nop();
        __nop();
        __nop();

        cycles = *DWT_CYCCNT;
        *DWT_CYCCNT = 0;        // reset the counter

        // record shortest number of instructions for loop
        totalCycles += cycles;
        if (cycles < minCycles)
            minCycles = cycles;
    }
}

void CoStkOverflowHook(OS_TID taskID)
{
    // Process stack overflow here
    while (1);
}

void setup(void)
{
    GPIO_InitTypeDef gpio;

    SystemCoreClockUpdate();

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);

    // Configure FPU
    FPU->FPCCR &= ~FPU_FPCCR_ASPEN_Msk;     // turn off FP context save
    FPU->FPCCR &= ~FPU_FPCCR_LSPEN_Msk;     // turn off lazy save

    // Status LED PB0, PB1
    digitalHi(LED_GPIO, LED0_PIN | LED1_PIN);
    gpio.GPIO_Pin = LED0_PIN | LED1_PIN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(LED_GPIO, &gpio);

    spiInit();
    vdacVoltage(0, 600);
}

#define MAIN_STACK_SIZE 512
OS_STK mainStack[MAIN_STACK_SIZE];

void mainTask(void *unused)
{
    while (1) {
        CoWaitForSingleFlag(osdData.osdUpdateFlag, 0);
        CoClearFlag(osdData.osdUpdateFlag);
        LED1_TOGGLE;
        CoTickDelay(5);
    }
}

int main(void)
{
    setup();

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    CoInitOS();

    osdInit();

    CoCreateTask(blinkTask, 0, 30, &blinkStack[BLINK_STACK_SIZE - 1], BLINK_STACK_SIZE);
    CoCreateTask(mainTask, 0, 10, &mainStack[MAIN_STACK_SIZE - 1], MAIN_STACK_SIZE);
    CoStartOS();

    return 0;
}
