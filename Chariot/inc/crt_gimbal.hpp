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
#include "alg_general.hpp"
#include "dvc_motor.hpp"
#include "para_gimbal.hpp"
#include <cstdint>

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

    enum IMUCalibrationMode : uint8_t {
        IMU_NOT_CALIBRATED = 0,
        IMU_CALIBRATED,
        IMU_CALIBRATING
    };


// private:
public:
    // 电机
    MotorLKMG *m_bigYawMotor;
    MotorGM6020 *m_smallYawMotor;

    // IMU
    IMU *imu;
    GSRLMath::Vector3f m_originEulerAngles;
    fp32 m_originSmallYawAngle;
    fp32 m_offsetSmallYawAngle;
    fp32 m_smallYawRealAngle;
    fp32 m_bigYawRealAngle;
    fp32 m_smallYawMotorEncoderAngle;
    fp32 m_bigYawMotorEncoderAngle;

    // IMU姿态校准
    bool m_isIMUCalibrated;
    fp32 m_lastIMUYawAngle;
    fp32 m_last2IMUYawAngle;
    fp32 m_last3IMUYawAngle;
    fp32 m_imuYawAngle;
    fp32 m_imuOffsetYawAngle;



    // 云台控制相关量
    GimbalMode m_gimbalMode;
    IMUCalibrationMode m_imuCalibrationMode;
    fp32 m_targetAngle; // 总云台指向角度

    fp32 m_bigYawControlTargetAngle; // 大电机目标角度
    fp32 m_smallYawControlTargetAngle; // 小电机目标角度
    fp32 m_mechDelta;

    // 遥控器
    Dr16RemoteControl m_remoteControl;

    // 标志位
    bool m_isInitComplete;

public:  
    Gimbal(MotorLKMG *bigYawMotor, MotorGM6020 *smallYawMotor, IMU *imuPtr);
    void init();
    void controlLoop();
    void imuGetYawAngleLoop();
    void receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage);
    void receiveRemoteControlDataFromISR(const uint8_t *rxData);

    void testSpining();
    void transmitMotorControlData();

    void testForRemoteControl();

private:
    void modeSelect();
    void modeControlLoop();
    void yawControl();
    void setSmallYawTargetAngle(const fp32 targetAngle);
    void setBigYawTargetAngle(const fp32 targetAngle);
    void setExternalControlTargetAngle(const fp32 bigYawTargetAngle, const fp32 smallYawTargetAngle);

    // 云台角度计算
    void getSmallYawMotorEncoderAngle();
    void getBigYawMotorEncoderAngle();
    void calculateSmallYawRealAngle();
    void calculateBigYawRealAngle();
    void calculateContoledAnglesData();
    void calculateAllAnglesData();

    
    // IMU
    void imuInit();
    void imuCalibration();
    void getIMUAttitude();

    // 遥控器
    void getTargetAngleFromRemoteControl();
    


};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
