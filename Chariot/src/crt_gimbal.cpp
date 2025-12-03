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

Gimbal::Gimbal(MotorDM4310 *yawMotor, MotorDM4310 *pitchMotor, MotorM2006 *rammerMotor, MotorM3508 *frictionLeftMotor, MotorM3508 *frictionRightMotor, IMU *imu)
    : m_yawMotor(yawMotor), m_pitchMotor(pitchMotor),
      m_rammerMotor(rammerMotor), m_frictionLeftMotor(frictionLeftMotor), m_frictionRightMotor(frictionRightMotor),
      m_imu(imu),
      m_gimbalMode(GIMBAL_NO_FORCE),
      m_yawTargetAngle(0.0f), m_pitchTargetAngle(0.0f),
      m_chassisMode(CHASSIS_NO_FORCE),
      m_gimbalTargetSpeed(0),
      m_chassisTargetSpeed(0),
      m_rammerState(false), m_frictionState(false),
      m_remoteControl(),
      m_isInitComplete(false) {}

void Gimbal::init()
{
    DWT_Init();
    CAN_Init(&hcan1, can1RxCallback);
    CAN_Init(&hcan2, can2RxCallback);
    UART_Init(&huart3, dr16RxCallback, 36);
    m_imu->init();
    m_isInitComplete = true;
}

void Gimbal::controlLoop()
{
    if (!m_isInitComplete) return;
    modeSelect();
    targetOrientationPlan();
    targetSpeedPlan();
    shootPlan();
    pitchControl();
    yawControl();
    shootControl();
    chassisControl();
    transmitGimbalMotorData();
    transmitChassisData();
}

void Gimbal::imuLoop()
{
    if (!m_isInitComplete) return;
    m_eulerAngle = m_imu->solveAttitude();
}

void Gimbal::receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (m_yawMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_pitchMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_rammerMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_frictionLeftMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_frictionRightMotor->decodeCanRxMessageFromISR(rxMessage)) return;
}

void Gimbal::receiveChassisDataFromISR(const can_rx_message_t *rxMessage)
{
    if (rxMessage->header.StdId == 0x601) {
        m_gameProgress    = rxMessage->data[0];
        m_leftShooterHeat = (uint16_t)rxMessage->data[2] | ((uint16_t)rxMessage->data[3] << 8);
        m_currentHP       = (uint16_t)rxMessage->data[5] | ((uint16_t)rxMessage->data[6] << 8);
    }
}

void Gimbal::receiveRemoteControlDataFromISR(const uint8_t *rxData)
{
    m_remoteControl.receiveRxDataFromISR(rxData);
}

void Gimbal::modeSelect()
{
    m_remoteControl.updateEvent();
    if (!m_remoteControl.isDr16RemoteControlConnected()) {
        m_gimbalMode  = GIMBAL_NO_FORCE;
        m_chassisMode = CHASSIS_NO_FORCE;
        return;
    }

    switch (m_remoteControl.getRightSwitchStatus()) {
        case Dr16RemoteControl::SWITCH_DOWN:
            m_gimbalMode  = GIMBAL_NO_FORCE;
            m_chassisMode = CHASSIS_NO_FORCE;
            if (m_remoteControl.getLeftSwitchEvent() == Dr16RemoteControl::SWITCH_TOGGLE_MIDDLE_UP) {
                m_gimbalMode = CALIBRATION;
            }
            break;

        case Dr16RemoteControl::SWITCH_MIDDLE:
            m_gimbalMode  = MANUAL_CONTROL;
            m_chassisMode = FOLLOW_GIMBAL;
            break;

        case Dr16RemoteControl::SWITCH_UP:
            m_gimbalMode  = AUTO_CONTROL;
            m_chassisMode = NO_FOLLOW;
            break;

        default:
            break;
    }
}

void Gimbal::targetOrientationPlan()
{
    switch (m_gimbalMode) {
        case MANUAL_CONTROL:
            setYawAngle(m_yawTargetAngle - rcStickDeadZoneFilter(m_remoteControl.getRightStickX()) * DT7_STICK_YAW_SENSITIVITY);
            setPitchAngle(m_pitchTargetAngle + rcStickDeadZoneFilter(m_remoteControl.getRightStickY()) * DT7_STICK_PITCH_SENSITIVITY);
            break;

        case AUTO_CONTROL:
            break;

        default:
            break;
    }
}

void Gimbal::targetSpeedPlan()
{
    switch (m_gimbalMode) {
        case MANUAL_CONTROL:
            m_gimbalTargetSpeed.x = rcStickDeadZoneFilter(m_remoteControl.getLeftStickX());
            m_gimbalTargetSpeed.y = rcStickDeadZoneFilter(m_remoteControl.getLeftStickY());
            if (m_chassisMode == NO_FOLLOW)
                m_gimbalTargetSpeed.z = rcStickDeadZoneFilter(m_remoteControl.getScrollWheel());
            break;

        case AUTO_CONTROL:
            break;

        default:
            break;
    }
}

void Gimbal::shootPlan()
{
    switch (m_gimbalMode)
    {
    case MANUAL_CONTROL:
        if (m_remoteControl.getLeftSwitchEvent() == Dr16RemoteControl::SWITCH_TOGGLE_MIDDLE_UP) {
            m_frictionState = !m_frictionState;
        }

        if ((m_remoteControl.getLeftSwitchStatus() == Dr16RemoteControl::SWITCH_DOWN) && m_frictionState && (m_leftShooterHeat < 350)) {
            m_rammerState = true;
        } else {
            m_rammerState = false;
        }
        break;

    case AUTO_CONTROL:
        break;
    
    default:
        break;
    }
}

void Gimbal::pitchControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_pitchMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制
            fp32 fdbData[2] = {GSRLMath::normalizeDeltaAngle(m_pitchTargetAngle - m_eulerAngle.y), -m_imu->getGyro().y};
            m_pitchMotor->externalClosedloopControl(0.0f, fdbData, 2);
            break;
        }

        default:
            break;
    }
}

void Gimbal::yawControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_yawTargetAngle = m_eulerAngle.z;
            m_yawMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            m_yawMotor->setMotorZeroPosition();
            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制
            fp32 fdbData[2] = {GSRLMath::normalizeDeltaAngle(m_eulerAngle.z - m_yawTargetAngle), m_imu->getGyro().z};
            m_yawMotor->externalClosedloopControl(0.0f, fdbData, 2);
            break;
        }

        default:
            break;
    }
}

void Gimbal::shootControl()
{
    if (m_gimbalMode == GIMBAL_NO_FORCE) {
        m_rammerState   = false;
        m_frictionState = false;
        m_frictionRightMotor->openloopControl(0.0f);
        m_frictionLeftMotor->openloopControl(0.0f);
        m_rammerMotor->openloopControl(0.0f);
        return;
    }

    if (m_frictionState) {
        m_frictionLeftMotor->angularVelocityClosedloopControl(-FRICTION_TARGET_ANGULAR_VELOCITY);
        m_frictionRightMotor->angularVelocityClosedloopControl(FRICTION_TARGET_ANGULAR_VELOCITY);
    } else {
        m_frictionLeftMotor->angularVelocityClosedloopControl(0.0f);
        m_frictionRightMotor->angularVelocityClosedloopControl(0.0f);
    }

    if (m_rammerState) {
        m_rammerMotor->angularVelocityClosedloopControl(-RAMMER_TARGET_ANGULAR_VELOCITY);
        rammerStuckControl();
    } else {
        m_rammerMotor->angularVelocityClosedloopControl(0.0f);
    }
}

void Gimbal::rammerStuckControl()
{
    static uint8_t rammerStuckState = 0; // 0: 正常 1: 疑似卡弹 2: 证实卡弹
    volatile static uint32_t rammerStuckTime = 0;
    switch (rammerStuckState)
    {
    case 0: // 正常
        if (abs(m_rammerMotor->getCurrentAngularVelocity()) < 1.0f) {
            rammerStuckTime = DWT->CYCCNT; // 获取当前时间戳
            rammerStuckState = 1;
        }
        break;

    case 1: // 疑似卡弹
        if (abs(m_rammerMotor->getCurrentAngularVelocity()) > 1.0f) {
            rammerStuckState = 0; // 解除卡弹状态
        } else if (((uint32_t)(DWT->CYCCNT - rammerStuckTime)) / ((fp32)(SystemCoreClock)) > RAMMER_STUCK_TIMEOUT) {
            rammerStuckTime = DWT->CYCCNT; // 获取当前时间戳
            rammerStuckState = 2;
        }
        break;

    case 2: // 证实卡弹
        m_rammerMotor->angularVelocityClosedloopControl(RAMMER_STUCK_REVERT_ANGULAR_VELOCITY);
        if (((uint32_t)(DWT->CYCCNT - rammerStuckTime)) / ((fp32)(SystemCoreClock)) > RAMMER_REVERT_TIME) {
            rammerStuckState = 0; // 解除卡弹状态
        }
        break;
    
    default:
        break;
    }
}

void Gimbal::chassisControl()
{
    switch (m_chassisMode) {
        case CHASSIS_NO_FORCE:
            m_chassisTargetSpeed = 0.0f;
            break;

        case NO_FOLLOW:
            convertGimbalTargetSpeedToChassisTargetSpeed();
            break;

        case FOLLOW_GIMBAL:
            m_gimbalTargetSpeed.z = GSRLMath::normalizeDeltaAngle(m_yawMotor->getCurrentAngle()) * CHASSIS_FOLLOW_KP;
            GSRLMath::constrain(m_gimbalTargetSpeed.z, 1.0f);
            convertGimbalTargetSpeedToChassisTargetSpeed();
            break;

        case SPINNING:
            m_gimbalTargetSpeed.z = 1.0f;
            convertGimbalTargetSpeedToChassisTargetSpeed();
            break;

        default:
            break;
    }
}

void Gimbal::transmitGimbalMotorData()
{
    HAL_CAN_AddTxMessage(&hcan1, m_yawMotor->getMotorControlHeader(), m_yawMotor->getMotorControlData(), NULL);
    HAL_CAN_AddTxMessage(&hcan1, m_pitchMotor->getMotorControlHeader(), m_pitchMotor->getMotorControlData(), NULL);
    HAL_CAN_AddTxMessage(&hcan2, m_rammerMotor->getMotorControlHeader(), m_rammerMotor->getMotorControlData(), NULL);
    HAL_CAN_AddTxMessage(&hcan2, m_frictionLeftMotor->getMotorControlHeader(), (*m_frictionLeftMotor + *m_frictionRightMotor).getMotorControlData(), NULL);
}

void Gimbal::transmitChassisData()
{
    CAN_TxHeaderTypeDef header;
    uint8_t txData[8];
    uint16_t tempX, tempY, tempZ;
    header.DLC   = 8;
    header.IDE   = CAN_ID_STD;
    header.RTR   = CAN_RTR_DATA;
    header.StdId = GIMBAL_TO_CHASSIS_CAN_ID;
    tempX        = GSRLMath::convertFloatToUint(m_chassisTargetSpeed.x, -1.0f, 1.0f, 16);
    tempY        = GSRLMath::convertFloatToUint(m_chassisTargetSpeed.y, -1.0f, 1.0f, 16);
    tempZ        = GSRLMath::convertFloatToUint(m_chassisTargetSpeed.z, -1.0f, 1.0f, 16);
    txData[0]    = (tempX >> 8);
    txData[1]    = tempX;
    txData[2]    = (tempY >> 8);
    txData[3]    = tempY;
    txData[4]    = (tempZ >> 8);
    txData[5]    = tempZ;
    txData[6]    = 0;
    txData[7]    = 0;
    HAL_CAN_AddTxMessage(&hcan2, &header, txData, NULL);
}

inline void Gimbal::setPitchAngle(const fp32 &targetAngle)
{
    // 限制俯仰角度
    if (targetAngle > PITCH_UPPER_LIMIT)
        m_pitchTargetAngle = PITCH_UPPER_LIMIT;
    else if (targetAngle < PITCH_LOWER_LIMIT)
        m_pitchTargetAngle = PITCH_LOWER_LIMIT;
    else
        m_pitchTargetAngle = targetAngle;
}

inline void Gimbal::setYawAngle(const fp32 &targetAngle)
{
    // 借用normalizeDeltaAngle函数将目标角度限制在-PI到PI之间
    m_yawTargetAngle = GSRLMath::normalizeDeltaAngle(targetAngle);
}

inline void Gimbal::convertGimbalTargetSpeedToChassisTargetSpeed()
{
    fp32 deltaAngle = GSRLMath::normalizeDeltaAngle(m_yawMotor->getCurrentAngle() - 0.0f);
    Matrix33f transCoordinateMatrix(Matrix33f::ROTATION, deltaAngle, {0.0f, 0.0f, 1.0f});
    m_chassisTargetSpeed = transCoordinateMatrix * m_gimbalTargetSpeed;
}

inline fp32 Gimbal::rcStickDeadZoneFilter(const fp32 &rcStickValue)
{
    if (rcStickValue > DT7_STICK_DEAD_ZONE)
        return (rcStickValue - DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else if (rcStickValue < -DT7_STICK_DEAD_ZONE)
        return (rcStickValue + DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else
        return 0.0f;
}
