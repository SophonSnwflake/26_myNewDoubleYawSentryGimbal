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
    using Vector3f  = GSRLMath::Vector3f;
    using Matrix33f = GSRLMath::Matrix33f;
    // 云台模式
    enum GimbalMode : uint8_t {
        GIMBAL_NO_FORCE = 0,
        CALIBRATION,
        MANUAL_CONTROL,
        AUTO_CONTROL
    };
    // 底盘模式
    enum ChassisMode : uint8_t {
        CHASSIS_NO_FORCE = 0,
        NO_FOLLOW,
        FOLLOW_GIMBAL,
        SPINNING
    };

private:
    // 电机
    MotorDM4310 *m_yawMotor;
    MotorDM4310 *m_pitchMotor;
    MotorM2006 *m_rammerMotor;
    MotorM3508 *m_frictionLeftMotor;
    MotorM3508 *m_frictionRightMotor;

    // IMU
    IMU *m_imu;
    Vector3f m_eulerAngle;

    // 云台控制相关量
    GimbalMode m_gimbalMode;
    fp32 m_yawTargetAngle;
    fp32 m_pitchTargetAngle;

    // 底盘控制相关量
    ChassisMode m_chassisMode;
    Vector3f m_gimbalTargetSpeed;  // 云台坐标系下的目标速度
    Vector3f m_chassisTargetSpeed; // 底盘坐标系下的目标速度

    // 发射机构相关量
    bool m_rammerState;   // false: 停止 true: 发射
    bool m_frictionState; // false: 停止 true: 启动

    // 遥控器
    Dr16RemoteControl m_remoteControl;

    // 标志位
    bool m_isInitComplete;

    // 下C板上发裁判系统数据
    uint8_t m_gameProgress;
    uint16_t m_leftShooterHeat;
    // uint16_t m_rightShooterHeat;
    uint16_t m_currentHP;

public:
    Gimbal(MotorDM4310 *yawMotor, MotorDM4310 *pitchMotor, MotorM2006 *rammerMotor, MotorM3508 *frictionLeftMotor, MotorM3508 *frictionRightMotor, IMU *imu);
    void init();
    void controlLoop();
    void imuLoop();
    void receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage);
    void receiveChassisDataFromISR(const can_rx_message_t *rxMessage);
    void receiveRemoteControlDataFromISR(const uint8_t *rxData);

private:
    void modeSelect();
    void targetOrientationPlan();
    void targetSpeedPlan();
    void shootPlan();
    void pitchControl();
    void yawControl();
    void shootControl();
    void rammerStuckControl();
    void chassisControl();
    void transmitGimbalMotorData();
    void transmitChassisData();
    inline void setPitchAngle(const fp32 &targetAngle);
    inline void setYawAngle(const fp32 &targetAngle);
    inline void convertGimbalTargetSpeedToChassisTargetSpeed();
    inline fp32 rcStickDeadZoneFilter(const fp32 &rcStickValue);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
