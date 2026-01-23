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
    transmitMotorControlData();
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
            calculateAllAnglesData();
            break;

        case CALIBRATION:
            imuCalibration();
            m_smallYawMotor->openloopControl(0);
            m_bigYawMotor->openloopControl(0);
            break;

        case MANUAL_CONTROL: {
            calculateAllAnglesData();
            setExternalControlTargetAngle(m_bigYawControlTargetAngle, m_smallYawControlTargetAngle);
            // testForBigYawMotorPID();
            fp32 speed = -(imu->getGyro().z);
            fp32 targetSpeed = bigYawPID.getOuterLoop().pidGetData().output;
            sendTsetMessageToPC(0xF1, m_targetAngle * 180 / 3.1415926535f, 0, 0, 0);
            sendTsetMessageToPC(0xF2, m_bigYawRealAngle * 180 / 3.1415926535f, 0, 0, 0);
            sendTsetMessageToPC(0xF3, targetSpeed, 0, 0, 0); 
            sendTsetMessageToPC(0xF4, speed, 0, 0, 0);

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
    // --- 可调参数 ---
    const fp32 kRelHalfLimit  = 1.0471975512f; // pi/3 = 60deg
    const fp32 kRelMargin     = 0.0872664626f; // 5deg
    const fp32 kRelSoftLimit  = kRelHalfLimit - kRelMargin;
    const fp32 kBigFollowGain = 0.10f;

    auto angDiff = [&](fp32 a, fp32 b) -> fp32 {
        return GSRLMath::normalizeDeltaAngle(a - b);
    };
    auto absf = [](fp32 x) -> fp32 { return (x >= 0.0f) ? x : -x; };

    // 输入：期望总云台指向角（全局航向）
    m_targetAngle = GSRLMath::normalizeDeltaAngle(m_targetAngle);

    // 测量：上/下部分全局航向角
    m_smallYawRealAngle = GSRLMath::normalizeDeltaAngle(m_smallYawRealAngle);
    m_bigYawRealAngle   = GSRLMath::normalizeDeltaAngle(m_bigYawRealAngle);

    // 1) 小云台：全局期望
    fp32 small_target_global = m_targetAngle;

    // 2) 大云台：一阶跟随（不再追求“尽量不动”）
    static bool s_inited     = false;
    static fp32 s_big_target = 0.0f;
    if (!s_inited) {
        s_big_target = m_bigYawRealAngle;
        s_inited     = true;
    }

    fp32 e_big   = angDiff(m_targetAngle, s_big_target);
    s_big_target = GSRLMath::normalizeDeltaAngle(s_big_target + kBigFollowGain * e_big);

    // 3) 目标相对角软限位：small_target_global - big_target
    fp32 rel_target = angDiff(small_target_global, s_big_target);
    if (absf(rel_target) > kRelSoftLimit) {
        fp32 sgn     = (rel_target >= 0.0f) ? 1.0f : -1.0f;
        s_big_target = GSRLMath::normalizeDeltaAngle(small_target_global - sgn * kRelSoftLimit);
    }

    // 4) 实际相对角软保护：small_real - big_real
    fp32 rel_meas = angDiff(m_smallYawRealAngle, m_bigYawRealAngle);
    if (absf(rel_meas) > kRelSoftLimit) {
        fp32 sgn     = (rel_meas >= 0.0f) ? 1.0f : -1.0f;
        s_big_target = GSRLMath::normalizeDeltaAngle(m_smallYawRealAngle - sgn * kRelSoftLimit);
    }

    // ============================================================
    // 关键新增：把“小云台全局目标角”转换成“小Yaw电机编码器目标角”
    //
    // 由你提供的关系可得：
    // rel = small - big = motorEnc - SMALL_YAW_ZERO_RAD
    // => motorEncTarget = SMALL_YAW_ZERO_RAD + relNeed
    //
    // relNeed 为让小云台对地达到 small_target_global 所需夹角：
    // relNeed = wrap(small_target_global - big_real)
    // ============================================================
    fp32 rel_need = angDiff(small_target_global, m_bigYawRealAngle);

    // 为了避免大云台还没跟上时，小云台电机直接顶死限位，这里对 rel_need 做软限位夹紧
    if (absf(rel_need) > kRelSoftLimit) {
        fp32 sgn = (rel_need >= 0.0f) ? 1.0f : -1.0f;
        rel_need = sgn * kRelSoftLimit;
    }

    // 电机目标角（与 m_smallYawMotorEncoderAngle 同一坐标定义）
    fp32 small_motor_target =
        GSRLMath::normalizeDeltaAngle(SMALL_YAW_ZERO_RAD + rel_need);


    // 输出：大云台仍用“全局目标角”，小云台改为“小电机角度目标”
    
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
void Gimbal::imuInit()
{
    // IMU初始化
    imu->init();
}

// IMU校准
void Gimbal::imuCalibration()
{
    if (m_isIMUCalibrated) return;
    static bool started   = false;
    static uint32_t t0_ms = 0;

    static uint16_t n   = 0;
    static float sumSin = 0.0f, sumCos = 0.0f;

    const uint32_t now = HAL_GetTick();

    if (!started) {
        t0_ms   = now;
        started = true;
    }

    if ((uint32_t)(now - t0_ms) < 357u) return;
    sumSin += sinf(m_originSmallYawAngle);
    sumCos += cosf(m_originSmallYawAngle);
    n++;
    if (n >= 2) {
        const float stableYaw = atan2f(sumSin, sumCos);
        m_imuOffsetYawAngle   = GSRLMath::normalizeDeltaAngle(stableYaw);
        m_isIMUCalibrated     = true;
        started               = false;
        n                     = 0;
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
    // 获取IMU姿态数据
    m_originEulerAngles = imu->getEulerAngle();
    m_originSmallYawAngle = m_originEulerAngles.z;
    m_originSmallYawAngle = GSRLMath::normalizeDeltaAngle(m_originEulerAngles.z);
}

/*****************************************遥控器相关 ******************************************/

void Gimbal::getTargetAngleFromRemoteControl()
{ 
    m_remoteControl.updateEvent();
    m_targetAngle += -m_remoteControl.getLeftStickX() * DT7_STICK_YAW_SENSITIVITY;
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
    const fp32 gyro_z      = imu->getGyro().z;                             // rad/s（注意单位）
    const fp32 small_rel_w = m_smallYawMotor->getCurrentAngularVelocity(); // rad/s，小yaw相对大yaw的角速度（同符号约定）

    // 大yaw真实角速度 = IMU角速度 - 小yaw相对角速度
    const fp32 big_w = gyro_z + small_rel_w;

    fp32 bigYawfdbData[2] = {
        GSRLMath::normalizeDeltaAngle(m_bigYawRealAngle - bigYawTargetAngle),
        -big_w // 你原来对 gyro.z 取负号，这里保持同样的反馈约定
    };
    


    
    // 小云台控制
    fp32 smallYawfdbData[2] = {GSRLMath::normalizeDeltaAngle(m_smallYawMotorEncoderAngle - smallYawTargetAngle), -(imu->getGyro().z)};
    m_bigYawMotor->externalClosedloopControl(0.0f, bigYawfdbData, 2, -(m_smallYawMotor->getControllerOutput() * BIG_YAW_WEIGHT_RATIO));
    m_smallYawMotor->externalClosedloopControl(0.0f, smallYawfdbData, 2,(m_bigYawMotor->getControllerOutput() * SMALL_YAW_WEIGHT_RATIO));
    m_smallYawMotorErrorAngle = GSRLMath::normalizeDeltaAngle(m_smallYawRealAngle - smallYawTargetAngle);
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

/*****************************************大YawPID测试 ******************************************/
void Gimbal::testForBigYawMotorPID()
{
    const fp32 gyro_z      = imu->getGyro().z;            // rad/s（注意单位）
    const fp32 small_rel_w = m_smallYawMotor->getCurrentAngularVelocity(); // rad/s，小yaw相对大yaw的角速度（同符号约定）

    // 大yaw真实角速度 = IMU角速度 - 小yaw相对角速度
    const fp32 big_w = gyro_z + small_rel_w;

    fp32 bigYawfdbData[2] = {
        GSRLMath::normalizeDeltaAngle(m_bigYawRealAngle - m_targetAngle),
        -big_w // 你原来对 gyro.z 取负号，这里保持同样的反馈约定
    };

    // m_bigYawMotor->externalClosedloopControl(0.0f, bigYawfdbData, 2);
    m_smallYawMotor->openloopControl(0);
}

void Gimbal::sendTsetMessageToPC(const uint8_t sbsbsbsbsbsb,fp32 yaw, fp32 pitch, fp32 roll,fp32 djws)
{
    // F1 帧：你自定义 DATA，这里放 3 个 int16（roll/pitch/yaw * 100）
    uint8_t FRAME_ID = sbsbsbsbsbsb; // 灵活格式帧范围：0xF1~0xFA :contentReference[oaicite:3]{index=3}
    constexpr uint8_t LEN      = 8;    // 4 * int16 = 8 字节（1~40）:contentReference[oaicite:4]{index=4}
    constexpr uint16_t TX_LEN  = 4 + LEN + 2;

    uint8_t txData[TX_LEN];

    txData[0] = 0xAA;     // HEAD :contentReference[oaicite:5]{index=5}
    txData[1] = 0xFF;     // D_ADDR（文档示例使用 0xFF）:contentReference[oaicite:6]{index=6}
    txData[2] = FRAME_ID; // ID
    txData[3] = LEN;      // LEN

    int16_t r = (int16_t)lroundf(roll * 100.0f);
    int16_t p = (int16_t)lroundf(pitch * 100.0f);
    int16_t y = (int16_t)lroundf(yaw * 100.0f);
    int16_t t = (int16_t)lroundf(djws * 100.0f); // reserv


        // DATA：小端（低字节在前）:contentReference[oaicite:7]{index=7}
        // DATA：4个 int16（小端），共 8 字节
    uint16_t ur = (uint16_t)r;
    uint16_t up     = (uint16_t)p;
    uint16_t uy     = (uint16_t)y;
    uint16_t ut     = (uint16_t)t;

    // r -> txData[4], [5]
    txData[4] = (uint8_t)(ur & 0xFF);        // 低字节
    txData[5] = (uint8_t)((ur >> 8) & 0xFF); // 高字节

    // p -> txData[6], [7]
    txData[6] = (uint8_t)(up & 0xFF);
    txData[7] = (uint8_t)((up >> 8) & 0xFF);

    // y -> txData[8], [9]
    txData[8] = (uint8_t)(uy & 0xFF);
    txData[9] = (uint8_t)((uy >> 8) & 0xFF);

    // t -> txData[10], [11]
    txData[10] = (uint8_t)(ut & 0xFF);
    txData[11] = (uint8_t)((ut >> 8) & 0xFF);

    // 计算 SC/AC：从 txData[0] 加到 DATA 结束（共 4+LEN 字节）:contentReference[oaicite:8]{index=8}
    uint16_t sc = 0;
    uint16_t ac = 0;
    for (uint8_t i = 0; i < (uint8_t)(4 + LEN); i++) {
        sc += txData[i];
        ac += sc;
    }

    txData[4 + LEN] = sc & 0xFF; // SC
    txData[5 + LEN] = ac & 0xFF; // AC

    HAL_UART_Transmit(&huart6, txData, TX_LEN, HAL_MAX_DELAY);
}
