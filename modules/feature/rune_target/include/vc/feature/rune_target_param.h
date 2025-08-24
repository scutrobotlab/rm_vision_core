/**
 * @file rune_target_param.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 神符靶心参数定义模块
 * @date 2025-08-24
 *
 * @details
 * 定义神符靶心参数结构体 RuneTargetParam，用于已激活与未激活靶心的面积、边长比、周长比、PNP 角点、缺口参数等。
 * 同时定义靶心绘制参数结构体 RuneTargetDrawParam，包括激活与未激活靶心的颜色、线宽、点半径及字体信息。
 */

#pragma once

#include "vc/core/yml_manager.hpp"
#include <opencv2/core/types.hpp>

//! 神符靶心参数模块
struct RuneTargetParam
{
    //--------------------[通用]------------------------
    //! 靶心半径 / mm，仅用于可视化
    float RADIUS = 150.f;

    //--------------------[已激活靶心]--------------------
    float ACTIVE_MIN_AREA = 60.f;                                //!< 最小面积
    float ACTIVE_MAX_AREA = 6000.f;                              //!< 最大面积
    float ACTIVE_MIN_SIDE_RATIO = 0.99f;                         //!< 最小边长比率
    float ACTIVE_MAX_SIDE_RATIO = 1.55f;                         //!< 最大边长比率
    float ACTIVE_MIN_AREA_RATIO = 0.8f;                          //!< 最小面积比率
    float ACTIVE_MAX_AREA_RATIO = 1.20f;                         //!< 最大面积比率
    float ACTIVE_MIN_PERI_RATIO = 0.35f;                         //!< 最小周长比率
    float ACTIVE_MAX_PERI_RATIO = 0.80f;                         //!< 最大周长比率
    float ACTIVE_MAX_CONVEX_AREA_RATIO = 0.9f;                   //!< 最大凸包面积比率
    float ACTIVE_MAX_CONVEX_PERI_RATIO = 0.11f;                  //!< 最大凸包周长比率
    float ACTIVE_MIN_AREA_RATIO_SUB = 0.70f;                     //!< 子轮廓最小面积比
    float ACTIVE_MAX_AREA_RATIO_SUB_TEN_RING = 0.30f;            //!< 十环时子轮廓面积最大比
    float ACTIVE_DEFAULT_SIDE = 60.f;                            //!< 默认边长
    std::vector<cv::Point3f> ACTIVE_3D = {cv::Point3f(0, 0, 0)}; //!< PNP 中心点 3D 坐标

    //--------------------[未激活靶心]--------------------
    float INACTIVE_MIN_AREA = 60.f;                                //!< 最小面积
    float INACTIVE_MAX_AREA = 6000.f;                              //!< 最大面积
    float INACTIVE_MIN_SIDE_RATIO = 0.99f;                         //!< 最小边长比率
    float INACTIVE_MAX_SIDE_RATIO = 1.55f;                         //!< 最大边长比率
    float INACTIVE_MIN_AREA_RATIO = 0.8f;                          //!< 最小面积比率
    float INACTIVE_MAX_AREA_RATIO = 1.20f;                         //!< 最大面积比率
    float INACTIVE_MIN_PERI_RATIO = 0.35f;                         //!< 最小周长比率
    float INACTIVE_MAX_PERI_RATIO = 0.80f;                         //!< 最大周长比率
    std::vector<cv::Point3f> INACTIVE_3D = {cv::Point3f(0, 0, 0)}; //!< PNP 中心点 3D 坐标

    //--------------------[缺口检测参数]--------------------
    float GAP_MIN_AREA_RATIO = 0.025f;       //!< 缺陷面积最小比
    float GAP_MAX_AREA_RATIO = 0.20f;        //!< 缺陷面积最大比
    float GAP_MIN_SIDE_RATIO = 1.55f;        //!< 缺陷最小长宽比
    float GAP_MAX_SIDE_RATIO = 8.0f;         //!< 缺陷最大长宽比
    float GAP_CIRCLE_RADIUS_RATIO = 0.7037f; //!< 缺陷圆半径与最外层圆半径比
    float GAP_MAX_DISTANCE_RATIO = 0.80f;    //!< 缺陷中心距最大比例
    float GAP_MIN_DISTANCE_RATIO = 0.50f;    //!< 缺陷中心距最小比例
    std::vector<cv::Point3d> GAP_3D = {      //!< PNP 缺口角点 3D 坐标
        cv::Point3d(-67.175, -67.175, 0), cv::Point3d(0, -95, 0),
        cv::Point3d(67.175, -67.175, 0), cv::Point3d(95, 0, 0),
        cv::Point3d(67.175, 67.175, 0), cv::Point3d(0, 95, 0),
        cv::Point3d(-67.175, 67.175, 0), cv::Point3d(-95, 0, 0)};

    cv::Matx33d ROTATION = cv::Matx33d::eye();         //!< 靶心坐标系相对于神符中心的旋转矩阵
    cv::Matx31d TRANSLATION = cv::Matx31d(0, -700, 0); //!< 靶心坐标系相对于神符中心的平移矩阵

    YML_INIT(
        RuneTargetParam,
        YML_ADD_PARAM(RADIUS);
        YML_ADD_PARAM(ACTIVE_MIN_AREA);
        YML_ADD_PARAM(ACTIVE_MAX_AREA);
        YML_ADD_PARAM(ACTIVE_MIN_SIDE_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_SIDE_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_PERI_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_PERI_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_CONVEX_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_CONVEX_PERI_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_RATIO_SUB);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_RATIO_SUB_TEN_RING);
        YML_ADD_PARAM(ACTIVE_DEFAULT_SIDE);
        YML_ADD_PARAM(ACTIVE_3D);
        YML_ADD_PARAM(INACTIVE_MIN_AREA);
        YML_ADD_PARAM(INACTIVE_MAX_AREA);
        YML_ADD_PARAM(INACTIVE_MIN_SIDE_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_SIDE_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_AREA_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_AREA_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_PERI_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_PERI_RATIO);
        YML_ADD_PARAM(INACTIVE_3D);
        YML_ADD_PARAM(GAP_MIN_AREA_RATIO);
        YML_ADD_PARAM(GAP_MAX_AREA_RATIO);
        YML_ADD_PARAM(GAP_MIN_SIDE_RATIO);
        YML_ADD_PARAM(GAP_MAX_SIDE_RATIO);
        YML_ADD_PARAM(GAP_CIRCLE_RADIUS_RATIO);
        YML_ADD_PARAM(GAP_MAX_DISTANCE_RATIO);
        YML_ADD_PARAM(GAP_MIN_DISTANCE_RATIO);
        YML_ADD_PARAM(GAP_3D);
        YML_ADD_PARAM(ROTATION);
        YML_ADD_PARAM(TRANSLATION););
};

inline RuneTargetParam rune_target_param; //!< 全局参数实例

//! 靶心绘制参数模块
struct RuneTargetDrawParam
{
    //! 已激活靶心绘制参数
    struct Active
    {
        cv::Scalar color = cv::Scalar(0, 255, 0); //!< 颜色
        int thickness = 2;                        //!< 线条粗细
        int point_radius = 3;                     //!< 点半径
        double default_circle_radius = 150.0;     //!< 默认圆圈大小

        YML_INIT(
            Active,
            YML_ADD_PARAM(color);
            YML_ADD_PARAM(thickness);
            YML_ADD_PARAM(point_radius);
            YML_ADD_PARAM(default_circle_radius););
    } active;

    //! 未激活靶心绘制参数
    struct Inactive
    {
        cv::Scalar color = cv::Scalar(0, 255, 0);          //!< 颜色
        int thickness = 2;                                 //!< 线条粗细
        int point_radius = 3;                              //!< 点半径
        double default_circle_radius = 150.0;              //!< 默认圆圈大小
        double font_scale = 0.5;                           //!< 角点标注字体大小
        int font_thickness = 1;                            //!< 角点标注字体粗细
        cv::Scalar font_color = cv::Scalar(255, 255, 255); //!< 角点标注字体颜色

        YML_INIT(
            Inactive,
            YML_ADD_PARAM(color);
            YML_ADD_PARAM(thickness);
            YML_ADD_PARAM(point_radius);
            YML_ADD_PARAM(default_circle_radius);
            YML_ADD_PARAM(font_scale);
            YML_ADD_PARAM(font_thickness);
            YML_ADD_PARAM(font_color););
    } inactive;
};

inline RuneTargetDrawParam rune_target_draw_param; //!< 全局绘制参数实例
