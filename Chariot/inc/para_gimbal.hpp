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
 *                            CAN ID
 ******************************************************************************/
#define GIMBAL_TO_CHASSIS_CAN_ID 0x501

/******************************************************************************
 *                            PID参数
 ******************************************************************************/
// 云台大Yaw电机
#define BIG_YAW_OUTER_KP             13.0f // 外环
#define BIG_YAW_OUTER_KI             0.8f
#define BIG_YAW_OUTER_KD             0.0f
#define BIG_YAW_OUTER_INTEGRAL_LIMIT 10.0f
#define BIG_YAW_OUTER_OUTPUT_LIMIT   1.0f
#define BIG_YAW_INNER_KP             0.8f // 内环
#define BIG_YAW_INNER_KI             0.0f
#define BIG_YAW_INNER_KD             0.0f
#define BIG_YAW_INNER_OUTPUT_LIMIT   10.0f
#define BIG_YAW_INNER_INTEGRAL_LIMIT 0.0f
// 云台小Yaw电机
#define SMALL_YAW_OUTER_KP             13.0f // 外环
#define SMALL_YAW_OUTER_KI             0.8f
#define SMALL_YAW_OUTER_KD             0.0f
#define SMALL_YAW_OUTER_INTEGRAL_LIMIT 10.0f
#define SMALL_YAW_OUTER_OUTPUT_LIMIT   1.0f
#define SMALL_YAW_INNER_KP             0.8f // 内环
#define SMALL_YAW_INNER_KI             0.0f
#define SMALL_YAW_INNER_KD             0.0f
#define SMALL_YAW_INNER_OUTPUT_LIMIT   10.0f
#define SMALL_YAW_INNER_INTEGRAL_LIMIT 0.0f
// 云台Pitch电机
#define PITCH_OUTER_KP                  15.0f // 外环
#define PITCH_OUTER_KI                  3.0f
#define PITCH_OUTER_KD                  0.0f
#define PITCH_OUTER_OUT_LIMIT           10.0f
#define PITCH_OUTER_IOUT_LIMIT          0.5f
#define PITCH_INNER_KP                  0.65f // 内环
#define PITCH_INNER_KI                  0.0f
#define PITCH_INNER_KD                  0.0f
#define PITCH_INNER_OUT_LIMIT           10.0f
#define PITCH_INNER_IOUT_LIMIT          0.0f
#define PITCH_INNER_LOWPASS_FILTER_PARA 0.9f
// 底盘跟随
#define CHASSIS_FOLLOW_KP 2.0f
// 摩擦轮
#define FRICTION_KP         300.0f
#define FRICTION_KI         10.0f
#define FRICTION_KD         0.0f
#define FRICTION_OUT_LIMIT  15000.0f
#define FRICTION_IOUT_LIMIT 2000.0f
// 拨弹轮
#define RAMMER_KP         3000.0f
#define RAMMER_KI         10.0f
#define RAMMER_KD         0.0f
#define RAMMER_OUT_LIMIT  8000.0f
#define RAMMER_IOUT_LIMIT 1000.0f

/******************************************************************************
 *                            电机参数
 ******************************************************************************/
#define LK_BIG_YAW_MOTOR_ID             1
#define BIG_YAW_ORIGIN_ENCODER_OFFSET   0
#define BIG_YAW_GEARBOX_RATIO           6
#define LK_SMALL_YAW_MOTOR_ID           1
#define SMALL_YAW_ORIGIN_ENCODER_OFFSET 0
#define SMALL_YAW_GEARBOX_RATIO         6

/******************************************************************************
 *                            IMU参数
 ******************************************************************************/
// Mahony算法参数
#define AHRS_AUTO_FREQ      0
#define AHRS_DEFAULT_FILTER 0
#define MAHONY_KP           0.8f
#define MAHONY_KI           0.0f
// IMU零飘补偿
#define GYRO_OFFSET_X  0.0f
#define GYRO_OFFSET_Y  0.0f
#define GYRO_OFFSET_Z  0.0f
#define ACCEL_OFFSET_X 0.0f
#define ACCEL_OFFSET_Y 0.0f
#define ACCEL_OFFSET_Z 0.0f
#define MAG_OFFSET_X   0.0f
#define MAG_OFFSET_Y   0.0f
#define MAG_OFFSET_Z   0.0f
// 安装朝向修正旋转矩阵
#define INSTALL_SPIN_MATRIX GSRLMath::Matrix33f::MatrixType::IDENTITY

/******************************************************************************
 *                            遥控器灵敏度与死区
 ******************************************************************************/
#define DT7_STICK_DEAD_ZONE         0.05f
#define DT7_STICK_PITCH_SENSITIVITY 0.02f
#define DT7_STICK_YAW_SENSITIVITY   0.05f

/******************************************************************************
 *                            云台角度限制
 ******************************************************************************/
#define PITCH_UPPER_LIMIT 0.2f
#define PITCH_LOWER_LIMIT -0.35f
/******************************************************************************
 *                            发射机构参数
 ******************************************************************************/
#define FRICTION_TARGET_ANGULAR_VELOCITY     720.0f
#define RAMMER_TARGET_ANGULAR_VELOCITY       2.0f * MATH_PI
#define RAMMER_STUCK_TIMEOUT                 1.0f
#define RAMMER_REVERT_TIME                   2.0f
#define RAMMER_STUCK_REVERT_ANGULAR_VELOCITY 1.0f * MATH_PI
