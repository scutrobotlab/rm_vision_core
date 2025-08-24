/**
 * @file dataio.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 数据输入输出模块
 * @date 2025-7-15
 */

#pragma once

#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

/**
 * @brief 陀螺仪数据结构
 *
 * @note 存储传感器测量的角度和角速度信息
 */
struct GyroData
{
    /**
     * @brief 转动姿态信息
     *
     * @note 包含偏转角、俯仰角、滚转角及其角速度
     */
    struct Rotation
    {
        float yaw = 0.f;         //!< 偏转角（向右运动为正）
        float pitch = 0.f;       //!< 俯仰角（向下运动为正）
        float roll = 0.f;        //!< 滚转角（顺时针运动为正）
        float yaw_speed = 0.f;   //!< 偏转角速度（向右运动为正）
        float pitch_speed = 0.f; //!< 俯仰角速度（向下运动为正）
        float roll_speed = 0.f;  //!< 滚转角速度（顺时针运动为正）
    } rotation;                  //!< 转动姿态实例
};
