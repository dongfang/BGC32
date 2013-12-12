/*

BGC32 from FocusFlight, a new alternative firmware
for the EvvGC controller

Original work Copyright (c) 2013 John Ihlein
                                 Alan K. Adamson

This file is part of BGC32.

Includes code and/or ideas from:

  1)BaseFlight
  2)EvvGC
  2)S.O.H. Madgwick

BGC32 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BGC32 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with EvvGC. If not, see <http://www.gnu.org/licenses/>.

*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

static uint16_t rc3pulseWidth = 3000;

static uint16_t rc4pulseWidth = 3000;

static uint16_t rc5pulseWidth = 3000;;

uint8_t rcActive = false;

///////////////////////////////////////////////////////////////////////////////

void rcInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;

    ///////////////////////////////////

    __disable_irq_nested();

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST,     ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);               // JTAG-DP Disabled and SW-DP Enabled

    //EXTI IN GPIO Config

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;  // PB3-Pitch, PB4-Roll, PB5-Yaw
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;                         // Set to Input Pull Down
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                      // GPIO Speed

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

    EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_InitStructure.EXTI_Line    = EXTI_Line3 | EXTI_Line4 | EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_ClearITPendingBit(EXTI_Line3 | EXTI_Line4 | EXTI_Line5);

    NVIC_EnableIRQ(EXTI3_IRQn);   // Enable interrupt
    NVIC_EnableIRQ(EXTI4_IRQn);   // Enable interrupt
    NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable interrupt

    ///////////////////////////////////

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 36 - 1; // 2 MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_Cmd(TIM3, ENABLE);

    __enable_irq_nested();
}

///////////////////////////////////////////////////////////////////////////////

void EXTI3_IRQHandler(void) //EXTernal interrupt routine PB3
{
    uint16_t diff;
    static uint16_t upTime3;

    EXTI->PR |= (1 << 3);  // Clear pending interrupt

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == 1)
        upTime3 = TIM3->CNT;
    else
    {
        diff = TIM3->CNT - upTime3;

        if ((diff > 1800) && (diff < 4200))
            rc3pulseWidth = diff;
        else
            rc3pulseWidth = 3000;

        rcActive = true;
    }
}

///////////////////////////////////////////////////////////////////////////////

void EXTI4_IRQHandler(void) //EXTernal interrupt routine PB4
{
    uint16_t diff;
    static uint16_t upTime4;

    EXTI->PR |= (1 << 4);  // Clear pending interrupt

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == 1)
        upTime4 = TIM3->CNT;
    else
    {
        diff = TIM3->CNT - upTime4;

        if ((diff > 1800) && (diff < 4200))
            rc4pulseWidth = diff;
        else
            rc4pulseWidth = 3000;

        rcActive = true;
    }
}

///////////////////////////////////////////////////////////////////////////////

void EXTI9_5_IRQHandler(void) //EXTernal interrupt routine PB5
{
    uint16_t diff;
    static uint16_t upTime5;

    EXTI->PR |= (1 << 5);  // clear pending interrupt

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 1)
        upTime5 = TIM3->CNT;
    else
    {
        diff = TIM3->CNT - upTime5;

        if ((diff > 1800) && (diff < 4200))
            rc5pulseWidth = diff;
        else
            rc5pulseWidth = 3000;

        rcActive = true;
    }
}

///////////////////////////////////////////////////////////////////////////////

uint16_t rxRead(uint8_t channel)
{
    if (channel == 0)       // Roll
        return rc4pulseWidth;
    else if (channel == 1)  // Pitch
        return rc3pulseWidth;
    else if (channel == 2)  // Yaw
        return rc5pulseWidth;
    else
        return 3000;
}

///////////////////////////////////////////////////////////////////////////////
