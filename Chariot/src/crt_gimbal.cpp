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
#include "tsk_isr.hpp"
#include "drv_misc.h"
#include <stdint.h>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            Gimbal类实现
 ******************************************************************************/

Gimbal::Gimbal(MotorLKMG *bigYawMotor, MotorGM6020 *smallYawMotor, IMU *imuPtr)
    : m_bigYawMotor(bigYawMotor),
      m_smallYawMotor(smallYawMotor),
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
    yawControl();
}

// 云台模式选择
void Gimbal::modeSelect()
{
    m_remoteControl.updateEvent();
    if (!m_remoteControl.isDr16RemoteControlConnected()) {
        m_gimbalMode  = GIMBAL_NO_FORCE;
        return;
    }

    if (m_remoteControl.getLeftSwitchStatus() == Dr16RemoteControl::SWITCH_DOWN) {
        m_gimbalMode = GIMBAL_NO_FORCE;
        return;
    }

    if (!m_isIMUCalibrated) {
        m_gimbalMode = CALIBRATION;
        return;
    }

    auto cur                  = m_remoteControl.getRightSwitchStatus();
    static decltype(cur) last = Dr16RemoteControl::SWITCH_MIDDLE; 

    // 只在“刚拨到 DOWN”的那一帧触发一次
    if (cur == Dr16RemoteControl::SWITCH_DOWN &&
        last != Dr16RemoteControl::SWITCH_DOWN) {
        m_isIMUCalibrated = false; 
    }

    last = cur;

    switch (cur) {
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

// 云台控制
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
            setExternalControlTargetAngle(m_bigYawControlTargetAngle, m_smallYawControlTargetAngle);
        } break;
        case AUTO_CONTROL: {
            break;
        }
        default:
            break;
    }
}

/******************************************数学计算******************************************/

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

void Gimbal::calculateSmallYawRealAngle()
{
    m_smallYawRealAngle =
        GSRLMath::normalizeDeltaAngle(m_originSmallYawAngle - m_imuOffsetYawAngle);
}

void Gimbal::calculateBigYawRealAngle()
{
    m_mechDelta =
        GSRLMath::normalizeDeltaAngle(SMALL_YAW_ZERO_RAD - m_smallYawMotorEncoderAngle);

    m_bigYawRealAngle =
        GSRLMath::normalizeDeltaAngle(m_smallYawRealAngle + m_mechDelta);
}

void Gimbal::calculateContoledAnglesData()
{
    // 新目标：小云台尽量快速、精准指向；不再刻意让大云台“尽量不动”
    // 策略：
    // 1) 小云台目标 = 期望全局航向（直接指向，最快响应）
    // 2) 大云台目标主动跟随期望（用一阶方式避免“目标瞬跳”）
    // 3) 同时对“小相对大”的机械限位做软约束（±60°留安全裕量），必要时强制大云台目标搬运以保护限位

    // --- 可调参数 ---
    const fp32 kRelHalfLimit  = 1.0471975512f; // pi/3 = 60deg（机械半限位）
    const fp32 kRelMargin     = 0.0872664626f; // 5deg 安全裕量（避免硬怼限位）
    const fp32 kRelSoftLimit  = kRelHalfLimit - kRelMargin;
    const fp32 kBigFollowGain = 0.10f; // 0~1：越大大云台越积极跟随（与控制周期相关）

    auto wrapPi = [](fp32 a) -> fp32 {
        return GSRLMath::normalizeDeltaAngle(a);
    };

    auto angDiff = [&](fp32 a, fp32 b) -> fp32 {
        // 最短有符号角差：wrap(a-b) 到 [-pi, pi]
        return wrapPi(a - b);
    };

    auto absf = [](fp32 x) -> fp32 { return (x >= 0.0f) ? x : -x; };

    // 输入：期望总云台指向角（全局航向）
    fp32 yaw_cmd = wrapPi(m_targetAngle);

    // 测量：上/下部分全局航向角
    fp32 yaw_small = wrapPi(m_smallYawRealAngle);
    fp32 yaw_big   = wrapPi(m_bigYawRealAngle);

    // 1) 小云台：直接指向期望（最快、最直）
    fp32 small_target_global = yaw_cmd;

    // 2) 大云台：主动跟随期望（不再追求“尽量不动”）
    //    用静态状态做一阶“目标跟随”，避免上电/突变时目标跳变太猛
    static bool s_inited     = false;
    static fp32 s_big_target = 0.0f;
    if (!s_inited) {
        s_big_target = yaw_big; // 从当前位置开始跟随，避免刚开始目标突跳
        s_inited     = true;
    }

    // 2.1 一阶跟随：big_target <- big_target + gain*(cmd - big_target)
    fp32 e_big   = angDiff(yaw_cmd, s_big_target);
    s_big_target = wrapPi(s_big_target + kBigFollowGain * e_big);

    // 3) 软限位保护（按“目标相对角”约束）：确保小云台相对大云台的目标不逼近±60°
    fp32 rel_target = angDiff(small_target_global, s_big_target); // small - big
    if (absf(rel_target) > kRelSoftLimit) {
        fp32 sgn = (rel_target >= 0.0f) ? 1.0f : -1.0f;
        // 强制把 big_target 搬到“刚好让 small 处于软限位边界”处
        s_big_target = wrapPi(small_target_global - sgn * kRelSoftLimit);
    }

    // 4) 额外安全（按“实际相对角”保护）：万一小云台跟不上/被外力推到限位附近
    fp32 rel_meas = angDiff(yaw_small, yaw_big); // 实际 small - big
    if (absf(rel_meas) > kRelSoftLimit) {
        fp32 sgn = (rel_meas >= 0.0f) ? 1.0f : -1.0f;
        // 让大云台向小云台方向追，尽快把相对角拉回软限位内
        s_big_target = wrapPi(yaw_small - sgn * kRelSoftLimit);
    }

    // 输出：写入你工程的控制目标（全局航向目标）
    setSmallYawTargetAngle(small_target_global);
    setBigYawTargetAngle(s_big_target);
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
void Gimbal::imuInit()
{
    // IMU初始化
    imu->init();
}

// IMU校准
void Gimbal::imuCalibration()
{
    // 如果你希望“已校准也允许重新校准”，把这句删掉，
    // 然后在进入 CALIBRATION 模式时手动 m_isIMUCalibrated = false;
    if (m_isIMUCalibrated) return;

    // ===== 可调参数 =====
    constexpr uint32_t kSettleMs   = 5000; // 静置等待时间：5秒
    constexpr uint32_t kAvgMs      = 200;  // 取平均时间：200ms
    constexpr uint32_t kGapResetMs = 200;  // 多久没调用就认为“中断了校准”，自动重置

    enum class State {
        WAIT_SETTLE,
        AVERAGING
    };

    // ===== 保留跨调用状态（但支持“中断自动重置”）=====
    static bool started     = false;
    static State state      = State::WAIT_SETTLE;
    static uint32_t t0_ms   = 0;
    static uint32_t last_ms = 0;
    static float sumSin     = 0.0f;
    static float sumCos     = 0.0f;

    const uint32_t now = HAL_GetTick();

    // 如果中间很久没调用（比如你切走模式了），下次回来从头开始计时
    if (started && (uint32_t)(now - last_ms) > kGapResetMs) {
        started = false;
    }
    last_ms = now;

    // 第一次进入（或被重置后）初始化
    if (!started) {
        started = true;
        state   = State::WAIT_SETTLE;
        t0_ms   = now;
        sumSin  = 0.0f;
        sumCos  = 0.0f;
        return; // 本轮只做初始化
    }

    // 这里用“滤波后的 yaw”。确保 m_originSmallYawAngle 每圈都更新过
    // 并且 normalizeDeltaAngle 是“返回值版”（你之前就是返回值版）
    const float yaw = GSRLMath::normalizeDeltaAngle(m_originSmallYawAngle);

    switch (state) {
        case State::WAIT_SETTLE:
            // 静置等待：让滤波器先跑热
            if ((uint32_t)(now - t0_ms) >= kSettleMs) {
                state  = State::AVERAGING;
                t0_ms  = now;
                sumSin = 0.0f;
                sumCos = 0.0f;
            }
            break;

        case State::AVERAGING:
            // 圆周平均，避免 ±pi 边界问题
            sumSin += sinf(yaw);
            sumCos += cosf(yaw);

            if ((uint32_t)(now - t0_ms) >= kAvgMs) {
                const float stableYaw = atan2f(sumSin, sumCos);
                m_imuOffsetYawAngle   = GSRLMath::normalizeDeltaAngle(stableYaw);
                m_isIMUCalibrated     = true;

                // 完成后清状态（防止后面误用/重复）
                started = false;
            }
            break;
    }
}

void Gimbal::imuGetYawAngleLoop()
{
    imu->solveAttitude();
    getIMUAttitude();
}

void Gimbal::getIMUAttitude()
{
    // 获取IMU姿态数据
    m_originEulerAngles = imu->getEulerAngle();
    m_originSmallYawAngle = m_originEulerAngles.z;
    m_originSmallYawAngle = GSRLMath::normalizeDeltaAngle(m_originEulerAngles.z);
}

/*****************************************遥控器相关 ******************************************/

void Gimbal::getTargetAngleFromRemoteControl()
{ 
    m_remoteControl.updateEvent();
    m_targetAngle += m_remoteControl.getLeftStickX() * DT7_STICK_YAW_SENSITIVITY;
    m_targetAngle = GSRLMath::normalizeDeltaAngle(m_targetAngle);
}

/*****************************************数据传输 ******************************************/
void Gimbal::transmitMotorControlData()
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
    HAL_CAN_AddTxMessage(&hcan1,
                         m_smallYawMotor->getMotorControlHeader(),
                         smallYawControlData,
                         &smallYawMailBox);
}
//调用外部闭环控制接口控制电机
void Gimbal::setExternalControlTargetAngle(const fp32 bigYawTargetAngle, const fp32 smallYawTargetAngle)
{
    fp32 bigYawfdbData[2] = {GSRLMath::normalizeDeltaAngle(m_bigYawRealAngle - bigYawTargetAngle), imu->getGyro().z};
    m_bigYawMotor->externalClosedloopControl(0.0f, bigYawfdbData, 2);
    // 小云台控制
    fp32 smallYawfdbData[2] = {GSRLMath::normalizeDeltaAngle(m_smallYawRealAngle - smallYawTargetAngle), imu->getGyro().z};
    m_smallYawMotor->externalClosedloopControl(0.0f, smallYawfdbData, 2);
}

void Gimbal::receiveRemoteControlDataFromISR(const uint8_t *rxData)
{
    m_remoteControl.receiveRxDataFromISR(rxData);
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