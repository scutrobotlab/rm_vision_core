/**
 * @file pose_node.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 位姿节点与坐标系类型定义头文件
 * @date 2025-7-15
 */

#pragma once
#include "transform6D.hpp"

/**
 * @brief 位姿节点类型别名
 */
using PoseNode = Transform6D; //!< 位姿节点类型

/**
 * @brief 坐标系类型
 */
struct CoordFrame
{
    static std::string WORLD;  //!< 世界坐标系标识
    static std::string CAMERA; //!< 相机坐标系标识
    static std::string JOINT;  //!< 转轴坐标系标识
    static std::string GYRO;   //!< 陀螺仪坐标系标识
};

//! 世界坐标系
inline std::string CoordFrame::WORLD = "world";
//! 相机坐标系
inline std::string CoordFrame::CAMERA = "camera";
//! 转轴坐标系
inline std::string CoordFrame::JOINT = "joint";
//! 陀螺仪坐标系
inline std::string CoordFrame::GYRO = "gyro";
