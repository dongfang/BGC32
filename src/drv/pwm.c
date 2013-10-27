/*
 *  pwm.c
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 */

#include "board.h"  // HJI

///////////////////////////////////////////////////////////////////////////////

int timer_1_8_deadtime_register = 200; // this is not just a delay value, check CPU reference manual for TIMx_BDTR DTG bit 0-7
int timer_4_5_deadtime_delay = 80;     // in 18MHz ticks

///////////////////////////////////////////////////////////////////////////////

void MaxCntClear(void)
{
    irqCnt[ROLL] = irqCnt[PITCH] = irqCnt[YAW] = 0;
    maxCnt[ROLL] = maxCnt[PITCH] = maxCnt[YAW] = 0;
    minCnt[ROLL] = minCnt[PITCH] = minCnt[YAW] = PWM_PERIODE + 1;
}

///////////////////////////////////////////////////////////////////////////////

static void Timer_Channel_Config(TIM_TypeDef *tim, TIM_OCInitTypeDef *OCInitStructure)
{
    TIM_OC1Init(tim, OCInitStructure);
    TIM_OC2Init(tim, OCInitStructure);
    TIM_OC3Init(tim, OCInitStructure);

    TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
}

///////////////////////////////////////////////////////////////////////////////

static void Timer_PWM_Advanced_Config(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;
    TIM_BDTRInitTypeDef         TIM_BDTRInitStructure;

    //Time Base configuration
    TIM_TimeBaseInitStructure.TIM_Prescaler = 3; // 18MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIODE;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseInitStructure);

    //Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = timer_1_8_deadtime_register;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(tim, &TIM_BDTRInitStructure);

    //Configuration in PWM mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    Timer_Channel_Config(tim, &TIM_OCInitStructure);
}

///////////////////////////////////////////////////////////////////////////////

static void Timer_PWM_General_Config(TIM_TypeDef *tim, int polarity)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;

    TIM_TimeBaseInitStructure.TIM_Prescaler = 3; // 18MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIODE;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = polarity;

    Timer_Channel_Config(tim, &TIM_OCInitStructure);
}

///////////////////////////////////////////////////////////////////////////////

static void SetupPWMIrq(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//Preemption Priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

///////////////////////////////////////////////////////////////////////////////

#define BB_PERIPH_ADDR(addr, bit) ((vu32*)(PERIPH_BB_BASE + ((void*)(addr)-(void*)PERIPH_BASE) * 32 + (bit) * 4))

void PWMConfig(void)
{
    MaxCntClear();

    //rewrite that thing;
    Timer_PWM_Advanced_Config(TIM1);
    Timer_PWM_Advanced_Config(TIM8);

    Timer_PWM_General_Config(TIM5, TIM_OCPolarity_High);
    Timer_PWM_General_Config(TIM4, TIM_OCPolarity_Low);

    TIM4->CNT = timer_4_5_deadtime_delay;
    TIM1->CNT = timer_4_5_deadtime_delay + 3 + PWM_PERIODE / 3;
    TIM8->CNT = timer_4_5_deadtime_delay + 5 + PWM_PERIODE * 2 / 3;

    SetupPWMIrq(TIM5_IRQn);    // yaw
    SetupPWMIrq(TIM1_UP_IRQn); // pitch
    SetupPWMIrq(TIM8_UP_IRQn); // roll

    __disable_irq();
    {
        /* code below is faster version of
        TIM_Cmd(TIM5, ENABLE);
        TIM_Cmd(TIM4, ENABLE);
        */
        vu32 *tim5Enable = BB_PERIPH_ADDR(&(TIM5->CR1), 0);
        vu32 *tim4Enable = BB_PERIPH_ADDR(&(TIM4->CR1), 0);
        vu32 *tim1Enable = BB_PERIPH_ADDR(&(TIM1->CR1), 0);
        vu32 *tim8Enable = BB_PERIPH_ADDR(&(TIM8->CR1), 0);
        *tim5Enable = 1;
        *tim4Enable = 1;
        *tim1Enable = 1;
        *tim8Enable = 1;
    }

    TIM_CtrlPWMOutputs(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    __enable_irq();
}

///////////////////////////////////////////////////////////////////////////////
