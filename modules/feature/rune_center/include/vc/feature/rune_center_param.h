/**
 * @file rune_center_param.h
 * @brief 神符中心参数头文件
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include <opencv2/core/types.hpp>
#include "vc/core/yml_manager.hpp"

/**
 * @brief 神符中心参数模块
 *
 * 包含神符中心的尺寸、圆形度、轮廓比率及坐标系变换矩阵等参数。
 */
struct RuneCenterParam
{
    float MAX_AREA = 1000.f;            //!< 最大面积
    float MIN_AREA = 10.f;              //!< 最小面积
    float MAX_SIDE_RATIO = 2.5f;        //!< 最大边长比率
    float MIN_SIDE_RATIO = 0.3f;        //!< 最小边长比率
    float MAX_SUB_AREA_RATIO = 0.2f;    //!< 子轮廓最大面积占比
    float MIN_CONVEX_AREA_RATIO = 0.9f; //!< 与凸包轮廓的最小面积占比
    float MAX_DEFECT_AREA_RATIO = 0.3f; //!< 最大缺陷的面积占比
    float MIN_ROUNDNESS = 0.2f;         //!< 最小圆形度
    float MAX_ROUNDNESS = 0.9f;         //!< 最大圆形度
    float MIN_AREA_FOR_RATIO = 20.f;    //!< 启用面积比例判断的最小面积
    float DEFAULT_SIDE = 20.f;          //!< 默认边长(用于强制构造时)

    double CENTER_CONCENTRICITY_RATIO = 0.08;                      //!< 父子轮廓同心度与最大轮廓的比值
    cv::Matx31d TRANSLATION = cv::Matx31d(0, 0, -165.44);          //!< 坐标系平移矩阵
    cv::Matx33d ROTATION = cv::Matx33d(1, 0, 0, 0, 1, 0, 0, 0, 1); //!< 坐标系旋转矩阵

    YML_INIT(
        RuneCenterParam,
        YML_ADD_PARAM(MAX_AREA);
        YML_ADD_PARAM(MIN_AREA);
        YML_ADD_PARAM(MAX_SIDE_RATIO);
        YML_ADD_PARAM(MIN_SIDE_RATIO);
        YML_ADD_PARAM(MAX_SUB_AREA_RATIO);
        YML_ADD_PARAM(MIN_CONVEX_AREA_RATIO);
        YML_ADD_PARAM(MIN_ROUNDNESS);
        YML_ADD_PARAM(MIN_AREA_FOR_RATIO);
        YML_ADD_PARAM(DEFAULT_SIDE);
        YML_ADD_PARAM(CENTER_CONCENTRICITY_RATIO);
        YML_ADD_PARAM(TRANSLATION);
        YML_ADD_PARAM(ROTATION););
};

inline RuneCenterParam rune_center_param; //!< 神符中心参数实例

/**
 * @brief 神符中心绘制参数
 *
 * 包含神符中心绘制时的颜色、线条粗细及默认半径。
 */
struct RuneCenterDrawParam
{
    cv::Scalar color = cv::Scalar(0, 255, 0); //!< 颜色
    int thickness = 2;                        //!< 线条粗细
    double default_radius = 150.0;            //!< 默认半径

    YML_INIT(
        RuneCenterDrawParam,
        YML_ADD_PARAM(color);
        YML_ADD_PARAM(thickness);
        YML_ADD_PARAM(default_radius););
};

inline RuneCenterDrawParam rune_center_draw_param; //!< 神符中心绘制参数实例
