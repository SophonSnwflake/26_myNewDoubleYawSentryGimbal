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
#include "Eigen/src/Geometry/Homogeneous.h"
#include "alg_general.hpp"
#include "dvc_motor.hpp"
#include "tsk_isr.hpp"
#include "drv_misc.h"
#include <cmath>
#include <cstdint>
#include <stdint.h>

extern CascadePID bigYawPID;

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            Gimbal类实现
 ******************************************************************************/

Gimbal::Gimbal(MotorLKMG *bigYawMotor, MotorGM6020 *smallYawMotor, MotorDM4310 *pitchMotor, IMU *imuPtr)
    : m_bigYawMotor(bigYawMotor),
      m_smallYawMotor(smallYawMotor),
      m_pitchMotor(pitchMotor),
      imu(imuPtr),
      m_gimbalMode(CALIBRATION),
      m_isInitComplete(false),
      m_isIMUCalibrated(false),
      m_remoteControl()
{
}

void Gimbal::init()
{
    DWT_Init();
    CAN_Init(&hcan1, can1RxCallback);
    CAN_Init(&hcan2, can2RxCallback);
    UART_Init(&huart3, dr16RxCallback, 36);
    // 初始化IMU
    if (!imu->init()) {
        return;
    }
    m_isInitComplete = true;
}

/******************************************控制逻辑******************************************/
void Gimbal::controlLoop()
{
    modeSelect();
    imuGetYawAngleLoop();
    getTargetAngleFromRemoteControl();
    pitchControl();
    yawControl();
    transmitMotorControlData();
}

// 云台模式选择
void Gimbal::modeSelect()
{
    // 更新遥控器状态
    m_remoteControl.updateEvent();
    //没连遥控器就无力
    if (!m_remoteControl.isDr16RemoteControlConnected()) {
        m_gimbalMode  = GIMBAL_NO_FORCE;
        return;
    }
    //左拨杆拨到下就是无力
    if (m_remoteControl.getLeftSwitchStatus() == Dr16RemoteControl::SWITCH_DOWN) {
        m_gimbalMode = GIMBAL_NO_FORCE;
        return;
    }
    // IMU没校准好就强制校准
    if (!m_isIMUCalibrated) {
        m_gimbalMode = CALIBRATION;
        return;
    }
    // 重置IMU校准状态以便重新校准
    auto currentSwitchMode= m_remoteControl.getRightSwitchStatus();
    static decltype(currentSwitchMode) lastSwitchMode = Dr16RemoteControl::SWITCH_MIDDLE; 
    if (currentSwitchMode == Dr16RemoteControl::SWITCH_DOWN &&
        lastSwitchMode != Dr16RemoteControl::SWITCH_DOWN) {
        m_isIMUCalibrated = false; 
    }
    lastSwitchMode = currentSwitchMode;
    // 根据右拨杆选择云台模式
    switch (currentSwitchMode) {
        case Dr16RemoteControl::SWITCH_DOWN:
            m_gimbalMode = CALIBRATION;
            break;
        case Dr16RemoteControl::SWITCH_MIDDLE:
            m_gimbalMode = MANUAL_CONTROL;
            break;
        case Dr16RemoteControl::SWITCH_UP:
            m_gimbalMode = AUTO_CONTROL;
            break;
        default:
            m_gimbalMode = GIMBAL_NO_FORCE;
            break;
    }
}

// 云台YAW轴控制
void Gimbal::yawControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_smallYawMotor->openloopControl(0);
            m_bigYawMotor->openloopControl(0);
            break;

        case CALIBRATION:
            imuCalibration();
            m_smallYawMotor->openloopControl(0);
            m_bigYawMotor->openloopControl(0);
            break;

        case MANUAL_CONTROL: {
            calculateAllAnglesData();
            setExternalPIDMessageToYawMotors();
        } break;
        case AUTO_CONTROL: {
            break;
        }
        default:
            break;
    }
}

//云台Pitch控制
void Gimbal::pitchControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_pitchMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            m_pitchMotor->openloopControl(0.0f);
            break;

        case MANUAL_CONTROL: {
            setPitchExternalPIDMessageToPitchMotor();
            break;
        }
        case AUTO_CONTROL: { 
            break;
        }
        default:
            break;
    }
}

// 云台Yaw外部反馈值闭环控制
void Gimbal::setExternalPIDMessageToYawMotors()
{
    const fp32 gyro_z      = imu->getGyro().z;                             
    const fp32 velocityBetweenSmallAndBig = m_smallYawMotor->getCurrentAngularVelocity();
    const fp32 bigYawRealVelocity       = gyro_z + velocityBetweenSmallAndBig;

    fp32 bigYawfdbData[2] = {
        GSRLMath::normalizeDeltaAngle(m_bigYawRealAngle - m_bigYawControlTargetAngle),
        -bigYawRealVelocity};
    fp32 smallYawfdbData[2] = {GSRLMath::normalizeDeltaAngle(m_smallYawMotorEncoderAngle - m_smallYawControlTargetAngle), -(imu->getGyro().z)};

    m_bigYawMotor->externalClosedloopControl(0.0f, bigYawfdbData, 2, -(m_smallYawMotor->getControllerOutput() * BIG_YAW_WEIGHT_RATIO));
    m_smallYawMotor->externalClosedloopControl(0.0f, smallYawfdbData, 2,(m_bigYawMotor->getControllerOutput() * SMALL_YAW_WEIGHT_RATIO));
    m_smallYawMotorErrorAngle = GSRLMath::normalizeDeltaAngle(m_smallYawRealAngle - m_smallYawControlTargetAngle);
}

// 云台Pitch外部反馈值闭环控制
void Gimbal::setPitchExternalPIDMessageToPitchMotor()
{
    if (m_targetPitchAngle > PITCH_UPPER_LIMIT)
    {
        m_targetPitchAngle = PITCH_UPPER_LIMIT;
    }
    else if (m_targetPitchAngle < PITCH_LOWER_LIMIT)
    {
        m_targetPitchAngle = PITCH_LOWER_LIMIT;
    }
    getPitchIMUAngle();
    fp32 pitchFdbData[2] = {GSRLMath::normalizeDeltaAngle(-m_pitchRealAngle + m_targetPitchAngle), imu->getGyro().y};
    m_pitchMotor->externalClosedloopControl(0.0f, pitchFdbData, 2, 0);
}

/******************************************角度获取及换算******************************************/

void Gimbal::calculateAllAnglesData()
{
    getSmallYawMotorEncoderAngle();
    getBigYawMotorEncoderAngle();
    calculateSmallYawRealAngle();
    calculateBigYawRealAngle();
    calculateContoledAnglesData();
}

void Gimbal::getSmallYawMotorEncoderAngle()
{
    m_smallYawMotorEncoderAngle = m_smallYawMotor->getCurrentAngle();
}

void Gimbal::getBigYawMotorEncoderAngle()
{
    m_bigYawMotorEncoderAngle = m_bigYawMotor->getCurrentAngle();
}

void Gimbal::getPitchIMUAngle()
{
    m_pitchRealAngle = imu->getEulerAngle().y;
}

void Gimbal::calculateSmallYawRealAngle()
{
    m_smallYawRealAngle =
        GSRLMath::normalizeDeltaAngle(m_originSmallYawAngle - m_imuOffsetYawAngle);
}

void Gimbal::calculateBigYawRealAngle()
{
    m_differenceValueBetweenSmallAndBigYaw =
        GSRLMath::normalizeDeltaAngle(SMALL_YAW_ZERO_RAD - m_smallYawMotorEncoderAngle);

    m_bigYawRealAngle =
        GSRLMath::normalizeDeltaAngle(m_smallYawRealAngle + m_differenceValueBetweenSmallAndBigYaw);
}

void Gimbal::calculateContoledAnglesData()
{
    const fp32 kRelHalfLimit  = 1.0471975512f; // pi/3 = 60deg
    const fp32 kRelMargin     = 0.0872664626f/4.0f; // 5deg
    const fp32 kRelSoftLimit  = kRelHalfLimit - kRelMargin;
    const fp32 kBigFollowGain = 0.10f;

    auto angDiff = [&](fp32 a, fp32 b) -> fp32 {
        return GSRLMath::normalizeDeltaAngle(a - b);
    };
    auto absf = [](fp32 x) -> fp32 { return (x >= 0.0f) ? x : -x; };

    m_targetYawAngle = GSRLMath::normalizeDeltaAngle(m_targetYawAngle); 
    m_smallYawRealAngle = GSRLMath::normalizeDeltaAngle(m_smallYawRealAngle);
    m_bigYawRealAngle   = GSRLMath::normalizeDeltaAngle(m_bigYawRealAngle);

    fp32 small_target_global = m_targetYawAngle;

    // 大云台一阶跟随
    static bool s_inited     = false;
    static fp32 s_big_target = 0.0f;
    if (!s_inited) {
        s_big_target = m_bigYawRealAngle;
        s_inited     = true;
    }

    fp32 e_big   = angDiff(m_targetYawAngle, s_big_target);
    s_big_target = GSRLMath::normalizeDeltaAngle(s_big_target + kBigFollowGain * e_big);

    // 软限位
    fp32 rel_target = angDiff(small_target_global, s_big_target);
    if (absf(rel_target) > kRelSoftLimit) {
        fp32 sgn     = (rel_target >= 0.0f) ? 1.0f : -1.0f;
        s_big_target = GSRLMath::normalizeDeltaAngle(small_target_global - sgn * kRelSoftLimit);
    }

    fp32 rel_meas = angDiff(m_smallYawRealAngle, m_bigYawRealAngle);
    if (absf(rel_meas) > kRelSoftLimit) {
        fp32 sgn     = (rel_meas >= 0.0f) ? 1.0f : -1.0f;
        s_big_target = GSRLMath::normalizeDeltaAngle(m_smallYawRealAngle - sgn * kRelSoftLimit);
    }
    fp32 rel_need = angDiff(small_target_global, m_bigYawRealAngle);

    // 软限位加紧
    if (absf(rel_need) > kRelSoftLimit) {
        fp32 sgn = (rel_need >= 0.0f) ? 1.0f : -1.0f;
        rel_need = sgn * kRelSoftLimit;
    }

    fp32 small_motor_target =
        GSRLMath::normalizeDeltaAngle(SMALL_YAW_ZERO_RAD + rel_need);

    setBigYawTargetAngle(s_big_target);
    setSmallYawTargetAngle(small_motor_target);
}

void Gimbal::setSmallYawTargetAngle(const fp32 targetAngle)
{
    m_smallYawControlTargetAngle = targetAngle;
}

void Gimbal::setBigYawTargetAngle(const fp32 targetAngle)
{
    m_bigYawControlTargetAngle = targetAngle;
}

/******************************************IMU相关 ******************************************/
// IMU初始化
void Gimbal::imuInit()
{
    imu->init();
}

// IMU校准
void Gimbal::imuCalibration()
{
    if (m_isIMUCalibrated) return;
    static bool started   = false;
    static uint32_t t0_ms = 0;
    static uint16_t calibrationTimes   = 0;
    static float sumSin = 0.0f, sumCos = 0.0f;
    const uint32_t now = HAL_GetTick();
    if (!started) {
        t0_ms   = now;
        started = true;
    }
    if ((uint32_t)(now - t0_ms) < IMU_CALIBRATION_WAITING_TIMES) return;
    sumSin += sinf(m_originSmallYawAngle);
    sumCos += cosf(m_originSmallYawAngle);
    calibrationTimes++;
    if (calibrationTimes >= IMU_CALIBRATION_STATISTICSING_TIMES) {
        const float stableYaw = atan2f(sumSin, sumCos);
        m_imuOffsetYawAngle   = GSRLMath::normalizeDeltaAngle(stableYaw);
        m_isIMUCalibrated     = true;
        started               = false;
        calibrationTimes     = 0;
        sumSin = sumCos = 0.0f;
    }
}

void Gimbal::imuGetYawAngleLoop()
{
    imu->solveAttitude();
    getIMUAttitude();
}

void Gimbal::getIMUAttitude()
{
    m_originEulerAngles = imu->getEulerAngle();
    m_originSmallYawAngle = m_originEulerAngles.z;
    m_originSmallYawAngle = GSRLMath::normalizeDeltaAngle(m_originEulerAngles.z);
}

/*****************************************遥控器相关 ******************************************/
void Gimbal::getTargetAngleFromRemoteControl()
{ 
    m_remoteControl.updateEvent();
    m_targetYawAngle += -m_remoteControl.getLeftStickX() * DT7_STICK_YAW_SENSITIVITY;
    m_targetYawAngle = GSRLMath::normalizeDeltaAngle(m_targetYawAngle);

    m_targetPitchAngle += m_remoteControl.getLeftStickY() * DT7_STICK_PITCH_SENSITIVITY;
    m_targetPitchAngle = GSRLMath::normalizeDeltaAngle(m_targetPitchAngle);
    if (m_targetPitchAngle > PITCH_UPPER_LIMIT) {
        m_targetPitchAngle = PITCH_UPPER_LIMIT;
    } else if (m_targetPitchAngle < PITCH_LOWER_LIMIT) {
        m_targetPitchAngle = PITCH_LOWER_LIMIT;
    }
}

/*****************************************数据传输 ******************************************/
void Gimbal::transmitMotorControlData()
{
    // CAN1发送大云台电机控制数据
    uint8_t bigYawControlData[8];
    memcpy(bigYawControlData, m_bigYawMotor->getMotorControlData(), sizeof(bigYawControlData));
    uint32_t bigYawMailBox;
    HAL_CAN_AddTxMessage(&hcan1,
                         m_bigYawMotor->getMotorControlHeader(),
                         bigYawControlData,
                         &bigYawMailBox);
    // CAN1发送云台Pitch电机控制数据
    uint8_t smallPitchControlData[8];
    memcpy(smallPitchControlData, m_pitchMotor->getMotorControlData(), sizeof(smallPitchControlData));
    uint32_t pitchMailBox;
    HAL_CAN_AddTxMessage(&hcan1,
                         m_pitchMotor->getMotorControlHeader(),
                         smallPitchControlData,
                         &pitchMailBox);
    // CAN2发送小云台电机控制数据
    uint8_t smallYawControlData[8];
    memcpy(smallYawControlData, m_smallYawMotor->getMotorControlData(), sizeof(smallYawControlData));
    uint32_t smallYawMailBox;
    HAL_CAN_AddTxMessage(&hcan2,
                         m_smallYawMotor->getMotorControlHeader(),
                         smallYawControlData,
                         &smallYawMailBox);
}

void Gimbal::receiveRemoteControlDataFromISR(const uint8_t *rxData)
{
    m_remoteControl.receiveRxDataFromISR(rxData);
}

void Gimbal::receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (m_smallYawMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_bigYawMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_pitchMotor->decodeCanRxMessageFromISR(rxMessage)) return;
}
