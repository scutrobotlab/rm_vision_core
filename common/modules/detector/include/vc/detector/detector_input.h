/**
 * @file detector_input.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 识别器输入参数定义文件
 * @date 2025-7-15
 */

#pragma once

#include "vc/core/yml_manager.hpp"
#include "vc/feature/feature_node.h"
#include "vc/dataio/dataio.h"

/**
 * @brief 识别器的输入参数
 *
 * @note 用于传递识别模块所需的输入信息，包括原始图像、时间戳、姿态信息以及其他辅助数据。
 */
struct DetectorInput
{
    /// @brief 输入图像（原始帧数据）
    DEFINE_PROPERTY(Image, public, public, (cv::Mat));

    /// @brief 时间戳（帧捕获时间，单位：微秒/毫秒，视实现而定）
    DEFINE_PROPERTY(Tick, public, public, (int64_t));

    /// @brief 陀螺仪信息（用于姿态补偿或预测）
    DEFINE_PROPERTY(GyroData, public, public, (GyroData));

    /// @brief 特征节点数组（可作为前一阶段处理结果的输入）
    DEFINE_PROPERTY(FeatureNodes, public, public, (std::vector<FeatureNode_ptr>));

    /// @brief 识别颜色（目标通道，如红/蓝）
    DEFINE_PROPERTY(Color, public, public, (PixChannel));
};
