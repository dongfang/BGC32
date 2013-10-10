/*
 *  pins.c
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis caat
 */
// HJI #include "pins.h"
// HJI #include "stm32f10x.h"
// HJI #include "utils.h"

#include "board.h"  // HJI

void GPIO_Config(void)  //Configures GPIO
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    ///////////////////////////////////////////////////////////////////////////

    // TIMER1 pin config

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    ///////////////////////////////////////////////////////////////////////////

    // Timer8 pin config

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    ///////////////////////////////////////////////////////////////////////////

    //Timer5 pin config

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    ///////////////////////////////////////////////////////////////////////////

    //Timer4 pin config

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    ///////////////////////////////////////////////////////////////////////////

    // HJI GPIO_InitStructure.GPIO_Pin = LED1_PIN;                   // LED1 Output Config
    // HJI GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    // HJI GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // HJI GPIO_Init(LED1_PORT, &GPIO_InitStructure);

    // HJI GPIO_InitStructure.GPIO_Pin = LED2_PIN;                   // LED2 Output Config
    // HJI GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    // HJI GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // HJI GPIO_Init(LED2_PORT, &GPIO_InitStructure);

    // HJI GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;  // Soft I2C Pin Config
    // HJI GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    // HJI GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // HJI GPIO_Init(GPIOB, &GPIO_InitStructure);
}
