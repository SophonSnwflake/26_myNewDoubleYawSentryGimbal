/**
 ******************************************************************************
 * @file           : para_gimbal.hpp
 * @brief          : 云台参数宏定义，方便修改调参
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
#include "math_const.h"

/* Defines -------------------------------------------------------------------*/
/******************************************************************************
 *                            PID参数
 ******************************************************************************/
// 云台大Yaw电机
#define BIG_YAW_OUTER_KP               3.25f
#define BIG_YAW_OUTER_KI               0.5f
#define BIG_YAW_OUTER_KD               4.0f
#define BIG_YAW_OUTER_INTEGRAL_LIMIT   200
#define BIG_YAW_OUTER_OUTPUT_LIMIT     300.0f
#define BIG_YAW_INNER_KP               100.0f 
#define BIG_YAW_INNER_KI               0.0f
#define BIG_YAW_INNER_KD               0.0f
#define BIG_YAW_INNER_INTEGRAL_LIMIT   200
#define BIG_YAW_INNER_OUTPUT_LIMIT     2000.0f
// 云台小Yaw电机
#define SMALL_YAW_OUTER_KP             10.0f // 外环（已固定）
#define SMALL_YAW_OUTER_KI             0.0f
#define SMALL_YAW_OUTER_KD             3.0f
#define SMALL_YAW_OUTER_INTEGRAL_LIMIT 10.0f
#define SMALL_YAW_OUTER_OUTPUT_LIMIT   200.0f
#define SMALL_YAW_INNER_KP             1000.0f 
#define SMALL_YAW_INNER_KI             0.0f
#define SMALL_YAW_INNER_KD             2.0f
#define SMALL_YAW_INNER_OUTPUT_LIMIT   100000.0f
#define SMALL_YAW_INNER_INTEGRAL_LIMIT 1111111.5f
// 云台Pitch电机
#define PITCH_OUTER_KP 11.0f // 外环11可以78545
#define PITCH_OUTER_KI                  1.0f
#define PITCH_OUTER_KD                  0.0f
#define PITCH_OUTER_OUT_LIMIT           7.5f
#define PITCH_OUTER_IOUT_LIMIT          0.5f
#define PITCH_INNER_KP                  0.4f 
#define PITCH_INNER_KI                  0.0f
#define PITCH_INNER_KD                  0.0f
#define PITCH_INNER_OUT_LIMIT           2.0f
#define PITCH_INNER_IOUT_LIMIT          0.0f
#define PITCH_OUTER_LOWPASS_FILTER_PARA 0.98f
#define PITCH_INNER_LOWPASS_FILTER_PARA 0.9f
/******************************************************************************
 *                            电机参数
 ******************************************************************************/
#define LK_BIG_YAW_MOTOR_ID             1
#define BIG_YAW_ORIGIN_ENCODER_OFFSET   0
#define BIG_YAW_GEARBOX_RATIO           1
#define LK_SMALL_YAW_MOTOR_ID           2 
#define SMALL_YAW_ORIGIN_ENCODER_OFFSET 0
#define SMALL_YAW_GEARBOX_RATIO         1
#define PITCH_MOTOR_CONTROL_ID          3

#define BIG_YAW_WEIGHT_RATIO            0.005f //大云台对小云台的惯性补偿权重
#define SMALL_YAW_WEIGHT_RATIO          0.10f  // 小云台对大云台的惯性补偿权重
#define SMALL_YAW_ZERO_RAD              1.4235 //小云台归中时的原始角度值
#define PITCH_MOTOR_ORIGIN_ANGLE_RAD    0.0f
/******************************************************************************
 *                            IMU及运动解算参数
 ******************************************************************************/
//ahrs
// IMU校准
#define IMU_CALIBRATION_WAITING_TIMES 357u // IMU校准等待循环次数
#define IMU_CALIBRATION_STATISTICSING_TIMES 3u // IMU校准统计循环次数
/******************************************************************************
 *                            遥控器灵敏度
 ******************************************************************************/
#define DT7_STICK_PITCH_SENSITIVITY 0.05f
#define DT7_STICK_YAW_SENSITIVITY   0.133f
/******************************************************************************
 *                            云台角度限制
 ******************************************************************************/
#define PITCH_UPPER_LIMIT 0.506f
#define PITCH_LOWER_LIMIT -0.32f
