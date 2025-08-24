#pragma once

#include "vc/core/yml_manager.hpp"

/**
 * @brief RuneFanParam 参数模块
 *
 * 神符扇叶识别参数，包括已激活与未激活扇叶的尺寸、面积、比例、3D坐标及坐标系变换矩阵等。
 */
struct RuneFanParam
{
    //------------------【已激活扇叶参数】-------------------
    float ACTIVE_MAX_SIDE_RATIO = 2.0;              //!< 已激活扇叶的最大长边短边比
    float ACTIVE_MIN_AREA = 100.0;                  //!< 已激活扇叶的最小面积
    float ACTIVE_MAX_AREA = 4000.0;                 //!< 已激活扇叶的最大面积
    float ACTIVE_MIN_AREA_RATIO = 0.05;             //!< 已激活扇叶面积占最小外接矩形比例下限
    float ACTIVE_MAX_AREA_RATIO = 0.60;             //!< 已激活扇叶面积占最小外接矩形比例上限
    float ACTIVE_MAX_AREA_PERIMETER_RATIO = 0.030;  //!< 已激活扇叶最大面积周长比
    float ACTIVE_MIN_AREA_PERIMETER_RATIO = 0.0002; //!< 已激活扇叶最小面积周长比

    std::vector<cv::Point3d> ACTIVE_TOP_3D = {cv::Point3d(-174, -32, 0), cv::Point3d(0, 0, 0), cv::Point3d(174, -32, 0)}; //!< 顶部角点3D坐标
    std::vector<cv::Point3d> ACTIVE_BOTTOM_CENTER_3D = {cv::Point3d(0, 350, 0)};                                          //!< 底部中心角点3D坐标
    std::vector<cv::Point3d> ACTIVE_SIDE_3D = {cv::Point3d(-186, 173, 0), cv::Point3d(186, 173, 0)};                      //!< 侧面角点3D坐标
    std::vector<cv::Point3d> ACTIVE_BOTTOM_SIDE_3D = {cv::Point3d(-57, 350, 0), cv::Point3d(57, 350, 0)};                 //!< 底部侧面角点3D坐标

    cv::Matx31d ACTIVE_TRANSLATION = cv::Matx31d(0, -505, 0); //!< 扇叶坐标系相对神符中心平移矩阵
    cv::Matx33d ACTIVE_ROTATION = cv::Matx33d(1, 0, 0,        //!< 扇叶坐标系相对神符中心旋转矩阵
                                              0, 1, 0,
                                              0, 0, 1);

    float ACTIVE_MIN_AREA_INCOMPLETE = 30.0;                 //!< 残缺扇叶最小面积
    float ACTIVE_MAX_AREA_PERIMETER_RATIO_INCOMPLETE = 0.07; //!< 残缺扇叶最大面积周长比
    float ACTIVE_MAX_DIRECTION_DELTA_INCOMPLETE = 45.0;      //!< 残缺扇叶方向最大偏移角度

    //-----------------【未激活扇叶参数】--------------------
    float INACTIVE_MAX_AREA_GROWTH_RATIO = 1.5f;                                                                                               //!< 匹配箭头时最大面积增长比例
    float INACTIVE_MAX_RECT_PROJECTION_RATIO = 1.5f;                                                                                           //!< 箭头匹配时矩形投影比值
    float INACTIVE_MIN_AREA_RATIO = 0.70;                                                                                                      //!< 灯臂凸包与最小外接矩形最小面积比例
    float INACTIVE_MERGE_DISTANCE_RATIO = 0.5;                                                                                                 //!< 轮廓合并距离阈值比例
    float INACTIVE_MAX_SIDE_RATIO = 6.0;                                                                                                       //!< 灯臂最大长边短边比
    float INACTIVE_MIN_SIDE_RATIO = 2.0;                                                                                                       //!< 灯臂最小长边短边比
    float INACTIVE_MIN_AREA = 100.0;                                                                                                           //!< 灯臂最小面积
    float INACTIVE_MAX_AREA = 4000.0;                                                                                                          //!< 灯臂最大面积
    float INACTIVE_MAX_DISTANCE_RATIO = 6.0;                                                                                                   //!< 灯臂与其他特征最大距离比例
    std::vector<cv::Point3d> INACTIVE_3D = {cv::Point3d(-30, 0, 0), cv::Point3d(30, 0, 0), cv::Point3d(30, 330, 0), cv::Point3d(-30, 330, 0)}; //!< 灯臂角点3D坐标
    cv::Matx31d INACTIVE_TRANSLATION = cv::Matx31d(0, -505, 0);                                                                                //!< 灯臂坐标系相对神符中心平移矩阵
    cv::Matx33d INACTIVE_ROTATION = cv::Matx33d(1, 0, 0,                                                                                       //!< 灯臂坐标系相对神符中心旋转矩阵
                                                0, 1, 0,
                                                0, 0, 1);

    YML_INIT(
        RuneFanParam,
        YML_ADD_PARAM(ACTIVE_MAX_SIDE_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_AREA);
        YML_ADD_PARAM(ACTIVE_MAX_AREA);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_PERIMETER_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_PERIMETER_RATIO);
        YML_ADD_PARAM(ACTIVE_TOP_3D);
        YML_ADD_PARAM(ACTIVE_BOTTOM_CENTER_3D);
        YML_ADD_PARAM(ACTIVE_SIDE_3D);
        YML_ADD_PARAM(ACTIVE_BOTTOM_SIDE_3D);
        YML_ADD_PARAM(ACTIVE_TRANSLATION);
        YML_ADD_PARAM(ACTIVE_ROTATION);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_INCOMPLETE);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_PERIMETER_RATIO_INCOMPLETE);
        YML_ADD_PARAM(ACTIVE_MAX_DIRECTION_DELTA_INCOMPLETE);
        YML_ADD_PARAM(INACTIVE_MAX_AREA_GROWTH_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_RECT_PROJECTION_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_AREA_RATIO);
        YML_ADD_PARAM(INACTIVE_MERGE_DISTANCE_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_SIDE_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_SIDE_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_AREA);
        YML_ADD_PARAM(INACTIVE_MAX_AREA);
        YML_ADD_PARAM(INACTIVE_MAX_DISTANCE_RATIO);
        YML_ADD_PARAM(INACTIVE_3D);
        YML_ADD_PARAM(INACTIVE_TRANSLATION);
        YML_ADD_PARAM(INACTIVE_ROTATION););
};

inline RuneFanParam rune_fan_param; //!< 全局RuneFan参数实例

/**
 * @brief 神符扇叶绘制参数
 */
struct RuneFanDrawParam
{
    /**
     * @brief 已激活扇叶绘制参数
     */
    struct Active
    {
        cv::Scalar color = cv::Scalar(0, 255, 0);           //!< 绘制颜色
        int thickness = 2;                                  //!< 线条粗细
        int point_radius = 3;                               //!< 点半径
        double font_scale = 0.5;                            //!< 文字大小
        int font_thickness = 1;                             //!< 文字粗细
        cv::Scalar font_color = cv::Scalar(255, 255, 255);  //!< 文字颜色
        int arrow_thickness = 2;                            //!< 箭头粗细
        double arrow_length = 50.0;                         //!< 箭头长度
        cv::Scalar arrow_color = cv::Scalar(255, 255, 255); //!< 箭头颜色

        YML_INIT(
            Active,
            YML_ADD_PARAM(color);
            YML_ADD_PARAM(thickness);
            YML_ADD_PARAM(point_radius);
            YML_ADD_PARAM(font_scale);
            YML_ADD_PARAM(font_thickness);
            YML_ADD_PARAM(font_color);
            YML_ADD_PARAM(arrow_thickness);
            YML_ADD_PARAM(arrow_length);
            YML_ADD_PARAM(arrow_color););
    } active;

    /**
     * @brief 未激活扇叶绘制参数
     */
    struct Inactive
    {
        cv::Scalar color = cv::Scalar(0, 255, 0);           //!< 绘制颜色
        int thickness = 2;                                  //!< 线条粗细
        int point_radius = 3;                               //!< 点半径
        double font_scale = 0.5;                            //!< 文字大小
        int font_thickness = 1;                             //!< 文字粗细
        cv::Scalar font_color = cv::Scalar(255, 255, 255);  //!< 文字颜色
        int arrow_thickness = 2;                            //!< 箭头粗细
        double arrow_length = 50.0;                         //!< 箭头长度
        cv::Scalar arrow_color = cv::Scalar(255, 255, 255); //!< 箭头颜色

        YML_INIT(
            Inactive,
            YML_ADD_PARAM(color);
            YML_ADD_PARAM(thickness);
            YML_ADD_PARAM(point_radius);
            YML_ADD_PARAM(font_scale);
            YML_ADD_PARAM(font_thickness);
            YML_ADD_PARAM(font_color);
            YML_ADD_PARAM(arrow_thickness);
            YML_ADD_PARAM(arrow_length);
            YML_ADD_PARAM(arrow_color););
    } inactive;
};

inline RuneFanDrawParam rune_fan_draw_param; //!< 全局RuneFan绘制参数实例
