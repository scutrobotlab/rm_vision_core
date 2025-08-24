/**
 * @file camera_param.hpp
 * @author SCUT RobotLab Vision Group
 * @brief 相机参数模块头文件
 * @date 2023-10-03
 */

#pragma once

#include <opencv2/core/types.hpp>
#include <vector>
#include <string>

#include "vc/core/yml_manager.hpp"

/**
 * @brief CameraParam 相机参数结构体
 *
 * 包含相机本体参数、畸变参数、坐标变换参数以及图传参数。
 */
struct CameraParam
{
    //! ---------------------相机本体参数--------------------
    int exposure = 800;    //!< 曝光
    int gamma = 100;       //!< Gamma 值
    int brightness = 15;   //!< 亮度
    int contrast = 100;    //!< 对比度
    int saturation = 100;  //!< 饱和度
    int sharpness = 100;   //!< 锐度
    int gain = 64;         //!< 全通道增益
    int b_gain = 100;      //!< 蓝色增益
    int g_gain = 100;      //!< 绿色增益
    int r_gain = 100;      //!< 红色增益
    int grab_mode = 1;     //!< Grab 模式
    int retrieve_mode = 1; //!< Retrieve 模式
    int auto_exposure = 0; //!< 自动曝光模式
    int auto_wb = 0;       //!< 自动白平衡模式

    //! 相机内参矩阵
    cv::Matx33f cameraMatrix = {1250, 0, 640, 0, 1250, 512, 0, 0, 1};

    //! 畸变参数
    cv::Matx<float, 5, 1> distCoeff = cv::Matx<float, 5, 1>(0, 0, 0, 0, 0);

    //! 相机坐标系到转轴坐标系的欧拉角
    cv::Matx<float, 3, 1> camera2joint_euler_angle = cv::Matx<float, 3, 1>(0, 0, 0);

    //! 相机坐标系到转轴坐标系的旋转矩阵
    cv::Matx<float, 3, 3> cam2joint_rmat = cv::Matx<float, 3, 3>(1, 0, 0, 0, 1, 0, 0, 0, 1);

    //! 相机坐标系到转轴坐标系的平移向量
    cv::Matx<float, 3, 1> cam2joint_tvec = cv::Matx<float, 3, 1>(0, 0, 0);

    //! 相机 LUT 查表
    std::vector<int> lut_vec;

    //! 相机序列号
    std::string serial_number;

    //! 相机分辨率
    int image_width = 1440;
    int image_height = 1080;

    //! ---------------------图传参数--------------------
    int trans_width = 720;  //!< 图传宽度
    int trans_height = 480; //!< 图传高度
    int hit_point_x = 360;  //!< 操作手击打点 X 坐标
    int hit_point_y = 240;  //!< 操作手击打点 Y 坐标

    //! 图传内参矩阵
    cv::Matx33f transMatrix = {1000, 0, 360, 0, 1000, 240, 0, 0, 1};

    //! 转轴坐标系到图传坐标系的欧拉角
    cv::Matx<float, 3, 1> trans2joint_euler_angle = cv::Matx<float, 3, 1>(0, 0, 0);

    //! 转轴坐标系到图传坐标系的旋转矩阵
    cv::Matx<float, 3, 3> trans2joint_rmat = cv::Matx<float, 3, 3>(1, 0, 0, 0, 1, 0, 0, 0, 1);

    //! 转轴坐标系到图传坐标系的平移向量
    cv::Matx<float, 3, 1> trans2joint_tvec = cv::Matx<float, 3, 1>(0, 0, 0);

    //! ---------------------YML 初始化--------------------
    YML_INIT(
        CameraParam,
        YML_ADD_PARAM(exposure);
        YML_ADD_PARAM(gamma);
        YML_ADD_PARAM(brightness);
        YML_ADD_PARAM(contrast);
        YML_ADD_PARAM(saturation);
        YML_ADD_PARAM(sharpness);
        YML_ADD_PARAM(gain);
        YML_ADD_PARAM(b_gain);
        YML_ADD_PARAM(g_gain);
        YML_ADD_PARAM(r_gain);
        YML_ADD_PARAM(grab_mode);
        YML_ADD_PARAM(retrieve_mode);
        YML_ADD_PARAM(auto_exposure);
        YML_ADD_PARAM(auto_wb);
        YML_ADD_PARAM(cameraMatrix);
        YML_ADD_PARAM(distCoeff);
        YML_ADD_PARAM(camera2joint_euler_angle);
        YML_ADD_PARAM(cam2joint_rmat);
        YML_ADD_PARAM(cam2joint_tvec);
        YML_ADD_PARAM(lut_vec);
        YML_ADD_PARAM(serial_number);
        YML_ADD_PARAM(image_width);
        YML_ADD_PARAM(image_height););
};

//! 全局 CameraParam 实例
inline CameraParam camera_param;
