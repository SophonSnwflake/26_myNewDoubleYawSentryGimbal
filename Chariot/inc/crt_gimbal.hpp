/**
 ******************************************************************************
 * @file           : crt_gimbal.hpp
 * @brief          : header file for crt_gimbal.cpp
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "GSRL.hpp"
#include "para_gimbal.hpp"

/* Exported types ------------------------------------------------------------*/

class Gimbal
{
public:
    // using Vector3f  = GSRLMath::Vector3f;
    // using Matrix33f = GSRLMath::Matrix33f;
    // 云台模式

    enum GimbalMode : uint8_t {
        GIMBAL_NO_FORCE = 0,
        CALIBRATION,
        MANUAL_CONTROL,
        AUTO_CONTROL
    };
    GSRLMath::Vector3f eulerAngles;

private:
    // 电机
    MotorLKMG *m_bigYawMotor;
    MotorLKMG *m_smallYawMotor;

    // IMU
    IMU *imu;

    // 云台控制相关量
    GimbalMode m_gimbalMode;

    // // 遥控器

    // 标志位
    bool m_isInitComplete;

public:
    Gimbal(MotorLKMG *bigYawMotor, MotorLKMG *smallYawMotor, IMU *imuPtr);
    void init();
    void controlLoop();
    void imuLoop();
    void receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage);
    void getIMUAttitude();

private:
    void yawControl();
    void setSmallYawTargetAngle(const fp32 targetAngle);
    void setBigYawTargetAngle(const fp32 targetAngle);
    void transmitMGMotorControlData();
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
