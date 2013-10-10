/*
  Sept 2013

  bgc32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Brushless Gimbal Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the EvvGC Brushless Gimbal Controller Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

#define RX_PULSE_1p5MS 3000  // 1.5 ms pulse width

uint8_t rcActive = false;

///////////////////////////////////////////////////////////////////////////////

static struct TIM_Channel { TIM_TypeDef *tim;
                            uint16_t channel;
                            uint16_t cc;
                          } Channels[] = { { TIM2, TIM_Channel_2, TIM_IT_CC2 },
                                           { TIM3, TIM_Channel_1, TIM_IT_CC1 },
                                           { TIM3, TIM_Channel_2, TIM_IT_CC2 }, };

static struct PWM_State { uint8_t  state;          // 0 = looking for rising edge, 1 = looking for falling edge
                          uint16_t riseTime;       // Timer value at rising edge of pulse
                          uint16_t pulseWidth;     // Computed pulse width
                        } Inputs[3] = { { 0, } };

static TIM_ICInitTypeDef  TIM_ICInitStructure = { 0, };


///////////////////////////////////////////////////////////////////////////////

static void parallelPWM_IRQHandler(TIM_TypeDef *tim)
{
    uint8_t i;
    uint16_t inputCaptureValue = 0;

    for (i = 0; i < 3; i++)
    {
        struct TIM_Channel channel = Channels[i];
        struct PWM_State *state = &Inputs[i];

        if (channel.tim == tim && (TIM_GetITStatus(tim, channel.cc) == SET))
        {
            TIM_ClearITPendingBit(channel.tim, channel.cc);
            if ( i == 0 )
                rcActive = true;

            switch (channel.channel)
            {
                case TIM_Channel_1:
                    inputCaptureValue = TIM_GetCapture1(channel.tim);
                    break;
                case TIM_Channel_2:
                    inputCaptureValue = TIM_GetCapture2(channel.tim);
                    break;
            }

            if (state->state == 0)
            {
                state->riseTime = inputCaptureValue;

                // switch states
                state->state = 1;

                TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
                TIM_ICInitStructure.TIM_Channel = channel.channel;
                TIM_ICInit(channel.tim, &TIM_ICInitStructure);
            }
            else
            {
                // inputCaptureValue has falling edge timer value

				// compute capture
				state->pulseWidth = inputCaptureValue - state->riseTime;

				// switch state
				state->state = 0;

				TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
				TIM_ICInitStructure.TIM_Channel = channel.channel;
                TIM_ICInit(channel.tim, &TIM_ICInitStructure);
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////

void TIM2_IRQHandler(void)
{
    parallelPWM_IRQHandler(TIM2);
}

///////////////////////////////////////////////////////////////////////////////

void TIM3_IRQHandler(void)
{
    parallelPWM_IRQHandler(TIM3);
}

///////////////////////////////////////////////////////////////////////////////

void rxInit(void)
{
    uint8_t i;

    GPIO_InitTypeDef         GPIO_InitStructure    = { 0, };
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = { 0, };
    NVIC_InitTypeDef         NVIC_InitStructure    = { 0, };

    // Parallel PWM Inputs
    // RX1  TIM2_CH2 PB3
    // RX2  TIM3_CH1 PB4
    // RX3  TIM3_CH2 PB5

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Input timers on TIM2 and TIM3 for PWM
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;

    NVIC_Init(&NVIC_InitStructure);

    // TIM2 and TIM3 timebase
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    TIM_TimeBaseStructure.TIM_Prescaler   = (36 - 1);
    TIM_TimeBaseStructure.TIM_Period      = 0xffff;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // PWM Input capture
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x0;

    for (i = 0; i < 3; i++)
    {
        TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
        TIM_ICInit(Channels[i].tim, &TIM_ICInitStructure);
    }

    TIM_ITConfig(TIM2, TIM_IT_CC2,              ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////

uint16_t rxRead(uint8_t channel)
{
    return Inputs[channel].pulseWidth;

}

///////////////////////////////////////////////////////////////////////////////
