#pragma once

// for roundf()
#define __USE_C99_MATH
#ifndef M_PI
#define M_PI    (3.1415926f)
#endif

// for REV2 hardware with videogen capability
#define REV2    1

/*
 rev 1
                   AF1             AF2       AF3         IO Str  Firmware Main    Firmware AF Note
PC13     CS1(F)    GPIO                                  TC      Flash CS
PC14     VBUS_SEN  EXTI15_10                             TC      VBUS Sense
PC15     ODD/EVEN  GPIO                                  TC      From LM1980
PA0                ADC1_IN1                              TTa     VBAT
PA1                ADC1_IN2                              TTa     OSDVBAT
PA2                ADC1_IN3        USART2_TX TIM15_CH1   TTa     ADC/PWM/USART
PA3                ADC1_IN4        USART2_RX TIM15_CH2   TTa     ADC/PWM/USART
PA4      DAC       DAC1_OUT1                             TTa     Audio Modem output
PA5                SPI1_SCK                              TTa     Flash/DAC SPI bus
PA6                SPI1_MISO                             TTa     Flash/DAC SPI bus
PA7                SPI1_MOSI                             TTa     Flash/DAC SPI bus
PB0      LED       GPIO            TIM3_CH3              TTa     Status LED
PB1      LED       GPIO            TIM3_CH4              TTa     Status LED
PB2      CS2(D)    GPIO                                  TTa     SPI DAC CS
PB10               USART3_TX                             TTa     USART3
PB11               USART3_RX                             TTa     USART3
PB12               NC                                    TTa     NC, empty pad
PB13     VIDEO     SPI2_SCK                              TTa     Pixel clock input
PB14     VIDEO     SPI2_MISO                             TTa     Pixel output, white(?)
PB15     VIDEO     TIM15_CH2                             TTa     VSync input
PA8      VIDEO     TIM1_CH1                              FT      HSync input
PA9                USART1_TX       I2C2_SCL              FT      USART1 or I2C2
PA10               USART1_RX       I2C2_SDA              FT      USART1 or I2C2
PA11     USB       USB_DM                                FT      USB
PA12     USB       USB_DP                                FT      USB
PA15     VIDEO     TIM8_CH1                              FT      Pixel clock generator
PB3      VIDEO     SPI3_SCK                              FT      Pixel clock input
PB4      VIDEO     SPI3_MISO                             FT      Pixel output, black(?)
PB5                TIM17_CH1                             FT      FLVS Input
PB6                TIM4_CH1                              FT      PPM_IN
PB7                TIM4_CH2                              FT      PWM1
PB8                CAN_RX                                FT      CAN Recive
PB9                CAN_TX                                FT      CAN Transmit

 rev 2
                   AF1             AF2       AF3         IO Str  Firmware Main    Firmware AF Note
PC13     CS1(F)    GPIO                                  TC      Flash CS
PC14     VBUS_SEN  EXTI15_10                             TC      VBUS Sense
PC15     ODD/EVEN  GPIO                                  TC      From LM1980
PA0                ADC1_IN1                              TTa     VBAT
PA1                ADC1_IN2                              TTa     OSDVBAT
PA2                ADC1_IN3        USART2_TX TIM15_CH1   TTa     ADC/PWM/USART
PA3                ADC1_IN4        USART2_RX TIM15_CH2   TTa     ADC/PWM/USART
PA4      DAC       DAC1_OUT1                             TTa     Audio Modem output
PA5                SPI1_SCK                              TTa     Flash/DAC SPI bus
PA6                SPI1_MISO                             TTa     Flash/DAC SPI bus
PA7                SPI1_MOSI                             TTa     Flash/DAC SPI bus
PB0      VGEN      TIM3_CH3                              TTa     CVBS_LO (black)
PB1      VGEN      TIM3_CH4                              TTa     CVBS_HI (white)
PB2      CS2(D)    GPIO                                  TTa     SPI DAC CS
PB10               USART3_TX                             TTa     USART3
PB11               USART3_RX                             TTa     USART3
PB12               GPIO                                  TTa     LED0
PB13     VIDEO     SPI2_SCK                              TTa     Pixel clock input
PB14     VIDEO     SPI2_MISO                             TTa     Pixel output, white(?)
PB15     VIDEO     TIM15_CH2                             TTa     VSync input
PA8      VIDEO     TIM1_CH1                              FT      HSync input
PA9                USART1_TX       I2C2_SCL              FT      USART1 or I2C2
PA10               USART1_RX       I2C2_SDA              FT      USART1 or I2C2
PA11     USB       USB_DM                                FT      USB
PA12     USB       USB_DP                                FT      USB
PA15     VIDEO     TIM8_CH1                              FT      Pixel clock generator
PB3      VIDEO     SPI3_SCK                              FT      Pixel clock input
PB4      VIDEO     SPI3_MISO                             FT      Pixel output, black(?)
PB5                TIM17_CH1                             FT      FLVS Input
PB6                TIM4_CH1                              FT      PPM_IN
PB7                GPIO            TIM4_CH2              FT      LED1
PB8                CAN_RX                                FT      CAN Recive
PB9                CAN_TX                                FT      CAN Transmit


Used DMA Channels
DMA1_Channel1       ADC1
DMA1_Channel2       USART3_TX or SPI1_RX
DMA1_Channel3       USART3_RX
DMA1_Channel4       line memcpy m2m DMA
DMA1_Channel5       SPI2_TX (required, white pixel)
DMA1_Channel6       memcpy m2m DMA (to copy black)

DMA2_Channel1       memset m2m DMA (to zero screen)
DMA2_Channel2       SPI3_TX (required, black pixel)
DMA2_Channel3       DAC1_OUT
DMA2_Channel4       memcpy m2m DMA (to copy lines)

Used timers (to be updated):
APB2: TIM1, TIM15, TIM16, TIM17
APB1: TIM2, TIM3, TIM4

Video switch truth table
SPI2 = A0, SPI3 = A1
A1 A0
 0  0 S1 (video_in)
 0  1 S2 (white)
 1  0 S3 (black)
 1  1 S4 (video_2)

*/


#include "stm32f30x.h"
#include <CoOS.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// Hardware definitions and GPIO
#define WHITE_SPI   SPI2
#define WHITE_DMA   DMA1_Channel5
#define BLACK_SPI   SPI3
#define BLACK_DMA   DMA2_Channel2
#define MEMSET_DMA  DMA2_Channel1
#define MEMCPY_DMA  DMA2_Channel4
#define BLKCPY_DMA  DMA2_Channel5

#define LED_GPIO    GPIOB
#ifndef REV2
// REV1 pinout
#define LED0_PIN    GPIO_Pin_0
#define LED1_PIN    GPIO_Pin_1
#else
// REV2 pinout
#define LED0_PIN    GPIO_Pin_12
#define LED1_PIN    GPIO_Pin_7
#endif

#define LED0_TOGGLE digitalToggle(LED_GPIO, LED0_PIN);
#define LED0_OFF    digitalHi(LED_GPIO, LED0_PIN);
#define LED0_ON     digitalLo(LED_GPIO, LED0_PIN);

#define LED1_TOGGLE digitalToggle(LED_GPIO, LED1_PIN);
#define LED1_OFF    digitalHi(LED_GPIO, LED1_PIN);
#define LED1_ON     digitalLo(LED_GPIO, LED1_PIN);

#include "drv_spi.h"
