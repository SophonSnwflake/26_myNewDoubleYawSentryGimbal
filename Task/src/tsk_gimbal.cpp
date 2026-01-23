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
#include "Define/def_bmi088.h"
#include "alg_pid.hpp"
#include "crt_gimbal.hpp"
#include "dvc_motor.hpp"
#include "dvc_imu.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Define --------------------------------------------------------------------*/
/******************************************************************************
 *                            电机相关
 ******************************************************************************/
/* PID ---------------------------------------------*/
CascadePID ::PIDParam bigYawOuterAnglePIDParam = { // 大yawPID参数
    BIG_YAW_OUTER_KP,
    BIG_YAW_OUTER_KI,
    BIG_YAW_OUTER_KD,
    BIG_YAW_OUTER_OUTPUT_LIMIT,
    BIG_YAW_OUTER_INTEGRAL_LIMIT};

CascadePID ::PIDParam bigYawInnerSpeedPIDParam = {

    BIG_YAW_INNER_KP,
    BIG_YAW_INNER_KI,
    BIG_YAW_INNER_KD,
    BIG_YAW_INNER_OUTPUT_LIMIT,
    BIG_YAW_INNER_INTEGRAL_LIMIT};

CascadePID ::PIDParam smallYawOuterAnglePIDParam = {
    // 小yawPID参数
    SMALL_YAW_OUTER_KP,
    SMALL_YAW_OUTER_KI,
    SMALL_YAW_OUTER_KD,
    SMALL_YAW_OUTER_OUTPUT_LIMIT,
    SMALL_YAW_OUTER_INTEGRAL_LIMIT};
CascadePID ::PIDParam smallYawInnerSpeedPIDParam = {
    SMALL_YAW_INNER_KP,
    SMALL_YAW_INNER_KI,
    SMALL_YAW_INNER_KD,
    SMALL_YAW_INNER_OUTPUT_LIMIT,
    SMALL_YAW_INNER_INTEGRAL_LIMIT};

CascadePID bigYawPID(bigYawOuterAnglePIDParam, bigYawInnerSpeedPIDParam, nullptr, nullptr);
CascadePID smallYawPID(smallYawOuterAnglePIDParam, smallYawInnerSpeedPIDParam, nullptr, nullptr);

/* Motor ---------------------------------------------*/
MotorLKMG bigYawMotor(LK_BIG_YAW_MOTOR_ID, &bigYawPID, BIG_YAW_ORIGIN_ENCODER_OFFSET, BIG_YAW_GEARBOX_RATIO);
MotorGM6020 smallYawMotor(LK_SMALL_YAW_MOTOR_ID, &smallYawPID, SMALL_YAW_ORIGIN_ENCODER_OFFSET);

/******************************************************************************
 *                            IMU相关
 ******************************************************************************/
Kalman_Quaternion_EKF myahrs(
    0.0f,   // 给定刷新频率，给0则为自动识别频率
    10.0f,  // Q的四元数过程噪声基准
    0.001f, // Q的零偏过程噪声基准
    1e7f,   // R，线加速度计噪声基准
    0.0f,   // 加速度计一阶低通滤波系数（0~1，越小滤得越重），默认0
    1       // 是否使用卡方检验（卡方检验阈值请使用类内函数接口来设定）
);

// 初始化IMU
BMI088::CalibrationInfo cali = {
    {0.0f, 0.0f, 0.0f}, // gyroOffset
    {0.0f, 0.0f, 0.0f}, // accelOffset
    {0.0f, 0.0f, 0.0f}, // magnetOffset
    {GSRLMath::Matrix33f::MatrixType::IDENTITY}};

// BMI088::SPIConfig accel{hspi1, GPIOA, GPIO_PIN_4};
// BMI088::SPIConfig gyro{hspi1, GPIOB, GPIO_PIN_0};

BMI088 imu(&myahrs,
           {&hspi1, GPIOA, GPIO_PIN_4},
           {&hspi1, GPIOB, GPIO_PIN_0},
           cali,
           nullptr,  // errorCallback（如果它是函数指针/可为空类型）
           nullptr); // magnet

Gimbal gimbal(&bigYawMotor, &smallYawMotor, &imu);
/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

fp32 Angle;

extern "C" void gimbal_task(void *argument)
{
    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 获取任务开始时间
    gimbal.init();
    while(1)
    while (1){
        gimbal.controlLoop();
        vTaskDelayUntil(&taskLastWakeTime, 5); // 确保任务以定周期5ms运行
    }
    }
