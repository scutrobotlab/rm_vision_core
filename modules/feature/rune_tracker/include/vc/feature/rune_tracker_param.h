/**
 * @file rune_tracker_param.h
 * @brief 神符追踪器参数模块头文件
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include "vc/core/yml_manager.hpp"

//! RuneTrackerParam 参数模块
struct RuneTrackerParam
{
    //! 追踪帧数
    int TRACK_FRAMES = 4;
    //! 采样时间间隔
    float SAMPLE_INTERVAL = 10.f;
    //! 角度滤波器过程噪声协方差矩阵
    cv::Matx22f ROTATE_Q = cv::Matx22f::eye();
    //! 角度滤波器测量噪声协方差矩阵
    cv::Matx22f ROTATE_R = cv::Matx22f::eye();
    //! 运动滤波器过程噪声协方差矩阵
    cv::Matx44f MOTION_Q = cv::Matx44f::eye();
    //! 运动滤波器测量噪声协方差矩阵
    cv::Matx44f MOTION_R = cv::Matx44f::diag({1, 1, 4, 4});
    //! 组合体队列的最大长度
    int MAX_DEQUE_SIZE = 32;

    YML_INIT(
        RuneTrackerParam,
        YML_ADD_PARAM(TRACK_FRAMES);
        YML_ADD_PARAM(SAMPLE_INTERVAL);
        YML_ADD_PARAM(ROTATE_Q);
        YML_ADD_PARAM(ROTATE_R);
        YML_ADD_PARAM(MOTION_Q);
        YML_ADD_PARAM(MOTION_R);
        YML_ADD_PARAM(MAX_DEQUE_SIZE););
};

//! RuneTracker 参数实例
inline RuneTrackerParam rune_tracker_param;
