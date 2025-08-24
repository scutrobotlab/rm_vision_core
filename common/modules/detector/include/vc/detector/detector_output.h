/**
 * @file detector_output.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 识别器输出参数定义文件
 * @date 2025-7-15
 */

#pragma once

#include "vc/core/yml_manager.hpp"
#include "vc/feature/feature_node.h"
#include "vc/dataio/dataio.h"

/**
 * @brief 识别器的输出参数
 *
 * @note 用于存储识别过程的结果，包括识别到的特征节点和识别结果的有效性。
 */
struct DetectorOutput
{
    /// @brief 识别结果（特征节点集合）
    DEFINE_PROPERTY(FeatureNodes, public, public, (std::vector<FeatureNode_ptr>));

    /// @brief 识别有效性标志
    DEFINE_PROPERTY(Valid, public, public, (bool));
};
