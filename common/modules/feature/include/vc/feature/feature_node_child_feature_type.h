/**
 * @file feature_node_child_feature_type.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 特征节点子特征类型定义文件
 * @date 2025-7-15
 */

#pragma once

#include <string>
#include "feature_node.h"

/**
 * @brief 子特征类型定义
 *
 * @note 用于标识不同类型的子特征节点，便于在特征节点管理中进行分类和查询。
 */
struct FeatureNode::ChildFeatureType
{
    static std::string RUNE_TARGET; ///< 神符靶心子特征类型
    static std::string RUNE_CENTER; ///< 神符中心子特征类型
    static std::string RUNE_FAN;    ///< 神符扇叶子特征类型
};

/// @brief 神符靶心子特征类型标识
inline std::string FeatureNode::ChildFeatureType::RUNE_TARGET = "rune_target";

/// @brief 神符中心子特征类型标识
inline std::string FeatureNode::ChildFeatureType::RUNE_CENTER = "rune_center";

/// @brief 神符扇叶子特征类型标识
inline std::string FeatureNode::ChildFeatureType::RUNE_FAN = "rune_fan";
