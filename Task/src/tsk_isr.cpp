/**
 ******************************************************************************
 * @file           : tsk_isr.cpp
 * @brief          : 中断服务程序
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "tsk_isr.hpp"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
extern Gimbal gimbal;

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/
void dr16RxCallback(uint8_t *Buffer, uint16_t Length)
{
    gimbal.receiveRemoteControlDataFromISR(Buffer);
}

volatile uint32_t can1_rx_cnt = 0, can2_rx_cnt = 0;

void can1RxCallback(can_rx_message_t *pRxMsg)
{
    can1_rx_cnt++;
    gimbal.receiveGimbalMotorDataFromISR(pRxMsg);
}

void can2RxCallback(can_rx_message_t *pRxMsg)       
{
    gimbal.receiveGimbalMotorDataFromISR(pRxMsg);
    // gimbal.receiveChassisDataFromISR(pRxMsg);
}