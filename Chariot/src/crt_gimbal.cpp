/**
 ******************************************************************************
 * @file           : crt_gimbal.cpp
 * @brief          : 云台控制
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "crt_gimbal.hpp"
#include "tsk_isr.hpp"
#include "drv_misc.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            Gimbal类实现
 ******************************************************************************/

Gimbal::Gimbal(MotorLKMG *bigYawMotor, MotorLKMG *smallYawMotor, IMU *imuPtr)
    : m_bigYawMotor(bigYawMotor),
      m_smallYawMotor(smallYawMotor),
      imu(imuPtr),
      m_gimbalMode(GIMBAL_NO_FORCE),
      m_isInitComplete(false)
{
}

void Gimbal::init()
{
    DWT_Init();
    CAN_Init(&hcan1, can1RxCallback);
    CAN_Init(&hcan2, can2RxCallback);
    // UART_Init(&huart3, dr16RxCallback, 36);
    m_isInitComplete = true;
}

void Gimbal::controlLoop()
{
    if (!m_isInitComplete) return;

    transmitMGMotorControlData();
}

/**
 * @brief 从中断服务程序接收云台电机CAN反馈数据
 * @param rxMessage CAN接收消息结构体指针
 */
void Gimbal::receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (m_smallYawMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_bigYawMotor->decodeCanRxMessageFromISR(rxMessage)) return;
}

void Gimbal::yawControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:

            break;

        case CALIBRATION:

            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制

            break;
        }

        default:
            break;
    }
}

void Gimbal::transmitMGMotorControlData()
{
    // 发送大云台电机控制数据
    uint8_t bigYawControlData[8];
    memcpy(bigYawControlData, m_bigYawMotor->getMotorControlData(), sizeof(bigYawControlData));
    uint32_t bigYawMailBox;
    HAL_CAN_AddTxMessage(&hcan1,
                         m_bigYawMotor->getMotorControlHeader(),
                         bigYawControlData,
                         &bigYawMailBox);

    // 发送小云台电机控制数据
    uint8_t smallYawControlData[8];
    memcpy(smallYawControlData, m_smallYawMotor->getMotorControlData(), sizeof(smallYawControlData));
    uint32_t smallYawMailBox;
    HAL_CAN_AddTxMessage(&hcan2,
                         m_smallYawMotor->getMotorControlHeader(),
                         smallYawControlData,
                         &smallYawMailBox);
}

void Gimbal::getIMUAttitude()
{
    // 获取IMU姿态数据
    eulerAngles = imu->getEulerAngle();
}   
