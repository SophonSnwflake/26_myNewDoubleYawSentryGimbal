/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : 测试任务
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

/* Define --------------------------------------------------------------------*/
/******************************************************************************
 *                            电机相关
 ******************************************************************************/
/* PID -----------------------------------------*/
// Yaw
CascadePID::PIDParam yawOuterParam = {
    YAW_OUTER_KP,  // Kp
    YAW_OUTER_KI,  // Ki
    YAW_OUTER_KD,   // Kd
    YAW_OUTER_OUT_LIMIT,  // outputLimit
    YAW_OUTER_IOUT_LIMIT    // intergralLimit
};
CascadePID::PIDParam yawInnerParam = {
    YAW_INNER_KP,  // Kp
    YAW_INNER_KI,  // Ki
    YAW_INNER_KD,   // Kd
    YAW_INNER_OUT_LIMIT,  // outputLimit
    YAW_INNER_IOUT_LIMIT    // intergralLimit
};
LowPassFilter<fp32> yawInnerLPF(YAW_INNER_LOWPASS_FILTER_PARA);
CascadePID yawPID(yawOuterParam, yawInnerParam, nullptr, &yawInnerLPF);
// Pitch
CascadePID::PIDParam pitchOuterParam = {
    PITCH_OUTER_KP,  // Kp
    PITCH_OUTER_KI,  // Ki
    PITCH_OUTER_KD,   // Kd
    PITCH_OUTER_OUT_LIMIT,  // outputLimit
    PITCH_OUTER_IOUT_LIMIT    // intergralLimit
};
CascadePID::PIDParam pitchInnerParam = {
    PITCH_INNER_KP,  // Kp
    PITCH_INNER_KI,  // Ki
    PITCH_INNER_KD,   // Kd
    PITCH_INNER_OUT_LIMIT,  // outputLimit
    PITCH_INNER_IOUT_LIMIT    // intergralLimit
};
LowPassFilter<fp32> pitchInnerLPF(PITCH_INNER_LOWPASS_FILTER_PARA);
CascadePID pitchPID(pitchOuterParam, pitchInnerParam, nullptr, &pitchInnerLPF);
// Friction
SimplePID::PIDParam frictionPIDParam = {
    FRICTION_KP,  // Kp
    FRICTION_KI,  // Ki
    FRICTION_KD,   // Kd
    FRICTION_OUT_LIMIT,  // outputLimit
    FRICTION_IOUT_LIMIT    // intergralLimit
};
SimplePID leftFrictionPID(SimplePID::PID_POSITION, frictionPIDParam);
SimplePID rightFrictionPID(SimplePID::PID_POSITION, frictionPIDParam);
// Rammer
SimplePID::PIDParam rammerPIDParam = {
    RAMMER_KP,  // Kp
    RAMMER_KI,  // Ki
    RAMMER_KD,   // Kd
    RAMMER_OUT_LIMIT,  // outputLimit
    RAMMER_IOUT_LIMIT    // intergralLimit
};
SimplePID rammerPID(SimplePID::PID_POSITION, rammerPIDParam);

/* Motor ---------------------------------------------*/
MotorDM4310 yawMotor(1, 0, 3.141593f, 40, 15, &yawPID);
MotorDM4310 pitchMotor(3, 2, 3.141593f, 40, 15, &pitchPID);
MotorM2006 rammerMotor(7, &rammerPID, 0, 36);
MotorM3508 leftFrictionMotor(1, &leftFrictionPID);
MotorM3508 rightFrictionMotor(3, &rightFrictionPID);

/******************************************************************************
 *                            IMU相关
 ******************************************************************************/
// AHRS算法
Mahony ahrs(AHRS_AUTO_FREQ, AHRS_DEFAULT_FILTER, MAHONY_KP, MAHONY_KI);
// IMU校准数据
BMI088::CalibrationInfo cali = {
    {GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z}, // gyroOffset
    {ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z}, // accelOffset
    {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z},  // magnetOffset
    {INSTALL_SPIN_MATRIX} // installSpinMatrix
};
// IMU类定义
BMI088 imu(&ahrs, {&hspi1, GPIOA, GPIO_PIN_4}, {&hspi1, GPIOB, GPIO_PIN_0}, cali);

// gimbal
Gimbal gimbal(&yawMotor, &pitchMotor, &rammerMotor, &leftFrictionMotor, &rightFrictionMotor, &imu);

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

extern "C" void gimbal_task(void *argument)
{
    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 获取任务开始时间
    gimbal.init();
    while(1)
    {
        gimbal.controlLoop();
        vTaskDelayUntil(&taskLastWakeTime, 5); // 确保任务以定周期1ms运行
    }
}
