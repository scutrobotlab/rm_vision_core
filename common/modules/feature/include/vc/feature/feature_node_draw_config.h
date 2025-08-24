/**
 * @file feature_node_draw_config.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 特征节点绘制配置定义文件
 * @date 2025-7-15
 */

#pragma once

#include "feature_node.h"

/**
 * @brief 特征节点绘制配置结构体
 *
 * @note 用于配置特征节点在图像上绘制时的样式和内容。
 */
struct FeatureNode::DrawConfig
{
    cv::Scalar color = cv::Scalar(100, 255, 0); ///< 绘制颜色（BGR 格式）
    int thickness = 2;                          ///< 绘制线条粗细
    DrawMask type = 0;                          ///< 绘制类型掩码，用于控制绘制层级
    bool draw_contours = false;                 ///< 是否绘制轮廓
    bool draw_corners = false;                  ///< 是否绘制角点
    bool draw_center = false;                   ///< 是否绘制中心点
    bool draw_pose_nodes = false;               ///< 是否绘制位姿节点
};
