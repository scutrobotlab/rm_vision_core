/**
 * @file rune_group_param.h
 * @brief RuneGroup 参数模块头文件
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include "vc/core/yml_manager.hpp"

//! RuneGroup 参数模块
struct RuneGroupParam
{
    template <typename _Tp, typename Enable>
    friend void load(_Tp &, const std::string &);

    using paraId = void;

    //! 神符扇叶间隔角度 (度)
    double INTERVAL_ANGLE = 72;

    //! 神符角度原始数据队列上限
    //! 为神符预测参数辨识预留足够样本
    size_t RAW_DATAS_SIZE = 500;

    //! 最大掉帧数
    int MAX_VANISH_NUMBER = 5;

    //! 两帧之间的最大 yaw 偏差 (度)
    double MAX_YAW_DEVIATION = 10.0;

    //! 两帧之间的最大 pitch 偏差 (度)
    double MAX_PITCH_DEVIATION = 10.0;

    //! 两帧之间的最大 roll 偏差 (度)
    double MAX_ROLL_DEVIATION = 10.0;

    //! 两帧之间的最大 x 偏差 (单位：毫米)
    double MAX_X_DEVIATION = 1000.0;

    //! 两帧之间的最大 y 偏差 (单位：毫米)
    double MAX_Y_DEVIATION = 1000.0;

    //! 两帧之间的最大 z 偏差 (单位：毫米)
    double MAX_Z_DEVIATION = 1000.0;

    //! 寻找待击打神符靶心的最大数据长度
    int MAX_GET_PENDING_DATA_LENGTH = 5;

    //! 是否启用神符位姿的极端值筛选
    bool ENABLE_EXTREME_VALUE_FILTER = true;

    //! 神符位姿筛选阈值——最大距离 (相机坐标系下，单位：毫米)
    double MAX_DISTANCE = 12000.0;

    //! 神符位姿筛选阈值——最小距离 (相机坐标系下，单位：毫米)
    double MIN_DISTANCE = 3000.0;

    //! 神符位姿筛选阈值——最大目标转角 yaw (度，相机坐标系下)
    double MAX_ROTATE_YAW = 60.0;

    //! 神符位姿筛选阈值——最大目标转角 pitch (度，相机坐标系下)
    double MAX_ROTATE_PITCH = 60.0;

    //! 神符位姿筛选阈值——最大朝向偏差角 yaw (度，相机坐标系下)
    double MAX_YAW_DEVIATION_ANGLE = 70.0;

    //! 神符位姿筛选阈值——最大朝向偏差角 pitch (度，相机坐标系下)
    double MAX_PITCH_DEVIATION_ANGLE = 50.0;

    YML_INIT(
        RuneGroupParam,
        YML_ADD_PARAM(INTERVAL_ANGLE);
        YML_ADD_PARAM(RAW_DATAS_SIZE);
        YML_ADD_PARAM(MAX_VANISH_NUMBER);
        YML_ADD_PARAM(MAX_YAW_DEVIATION);
        YML_ADD_PARAM(MAX_PITCH_DEVIATION);
        YML_ADD_PARAM(MAX_ROLL_DEVIATION);
        YML_ADD_PARAM(MAX_X_DEVIATION);
        YML_ADD_PARAM(MAX_Y_DEVIATION);
        YML_ADD_PARAM(MAX_Z_DEVIATION);
        YML_ADD_PARAM(MAX_GET_PENDING_DATA_LENGTH);
        YML_ADD_PARAM(ENABLE_EXTREME_VALUE_FILTER);
        YML_ADD_PARAM(MAX_DISTANCE);
        YML_ADD_PARAM(MIN_DISTANCE);
        YML_ADD_PARAM(MAX_ROTATE_YAW);
        YML_ADD_PARAM(MAX_ROTATE_PITCH);
        YML_ADD_PARAM(MAX_YAW_DEVIATION_ANGLE);
        YML_ADD_PARAM(MAX_PITCH_DEVIATION_ANGLE););
};

//! RuneGroup 参数模块全局实例
inline RuneGroupParam rune_group_param;
