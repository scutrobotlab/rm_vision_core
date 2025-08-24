#pragma once

#include "vc/core/yml_manager.hpp"

//! RuneDetectorParam 参数模块
struct RuneDetectorParam
{
    //! 二值化R-B阈值
    int GRAY_THRESHOLD_RED = 80;
    //! 二值化B-R阈值
    int GRAY_THRESHOLD_BLUE = 80;
    //! 能够参与神符特征匹配的最小面积
    double MIN_CONTOUR_AREA = 20;
    //! 能够参与神符特征匹配的最大面积
    double MAX_CONTOUR_AREA = 20000;
    //! 判断为已激活扇叶的面积阈值
    double BIG_ACTIVE_FAN_AREA = 1000;
    //! 已激活的神符扇叶的同心度与最小面积矩形的长度的比值
    double ACTIVE_FAN_CONCENTRICITY_RATIO = 0.08;
    //! 新匹配神符与已有神符的最小夹角（DEG）(正视时，两个神符的夹角应该为72°)
    double MIN_FORCE_CONSTRUCT_ANGLE = 45;

    //----------各个神符特征的识别准确概率-----------

    //! 未激活靶心的识别准确概率
    double INACTIVE_TARGET_ACCURACY = 0.95;
    //! 已激活靶心的识别准确概率
    double ACTIVE_TARGET_ACCURACY = 0.8;
    //! 未激活扇叶的识别准确概率
    double INACTIVE_FAN_ACCURACY = 0.95;
    //! 未激活扇叶的最低识别准确概率
    double MIN_INACTIVE_FAN_ACCURACY = 0.8;
    //! 已激活扇叶的识别准确概率
    double ACTIVE_FAN_ACCURACY = 0.95;
    //! 神符中心允许的的识别准确概率阈值
    double MIN_CENTER_ACCURACY = 0.80;

    //! 距离神符中心的最远距离比值
    double MAX_DISTANCE_RATIO = 1.5;

    //-----------神符组合体与追踪器最大匹配偏差比例-----------
    double MAX_MATCH_DEVIATION_RATIO = 0.2;
    //! 估计神符中心的有效时间(单位：秒)
    double ESTIMATE_CENTER_VALID_TIME = 4.0;
    //! 神符中心强制构造窗口的范围比例
    double CENTER_FORCE_CONSTRUCT_WINDOW_RATIO = 0.2;
    //! 是否启用神符中心强制构造窗口
    bool ENABLE_CENTER_FORCE_CONSTRUCT_WINDOW = true;

    YML_INIT(
        RuneDetectorParam,
        YML_ADD_PARAM(GRAY_THRESHOLD_RED);
        YML_ADD_PARAM(GRAY_THRESHOLD_BLUE);
        YML_ADD_PARAM(MIN_CONTOUR_AREA);
        YML_ADD_PARAM(MAX_CONTOUR_AREA);
        YML_ADD_PARAM(BIG_ACTIVE_FAN_AREA);
        YML_ADD_PARAM(ACTIVE_FAN_CONCENTRICITY_RATIO);
        YML_ADD_PARAM(MIN_FORCE_CONSTRUCT_ANGLE);
        YML_ADD_PARAM(INACTIVE_TARGET_ACCURACY);
        YML_ADD_PARAM(ACTIVE_TARGET_ACCURACY);
        YML_ADD_PARAM(INACTIVE_FAN_ACCURACY);
        YML_ADD_PARAM(MIN_INACTIVE_FAN_ACCURACY);
        YML_ADD_PARAM(ACTIVE_FAN_ACCURACY);
        YML_ADD_PARAM(MIN_CENTER_ACCURACY);
        YML_ADD_PARAM(MAX_DISTANCE_RATIO);
        YML_ADD_PARAM(MAX_MATCH_DEVIATION_RATIO);
        YML_ADD_PARAM(ESTIMATE_CENTER_VALID_TIME);
        YML_ADD_PARAM(CENTER_FORCE_CONSTRUCT_WINDOW_RATIO);
        YML_ADD_PARAM(ENABLE_CENTER_FORCE_CONSTRUCT_WINDOW);)
};

//! RuneDetectorParam 参数模块
inline RuneDetectorParam rune_detector_param;
