/**
 * @file feature_node.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 特征节点基类定义文件
 * @date 2025-7-15
 */

#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <memory>
#include <unordered_map>

#include "vc/core/property_wrapper.hpp"
#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/math/pose_node.hpp"
#include "vc/core/type_utils.h"
#include "vc/dataio/dataio.h"

#define FEATURE_NODE_DEBUG 1

/**
 * @brief 特征节点基类
 *
 * @note 用于承载视觉识别系统中的通用特征信息，支持图像缓存、位姿缓存、
 *       子特征节点管理及可选的绘制功能。
 */
class FeatureNode : public std::enable_shared_from_this<FeatureNode>
{
    /// @brief 智能指针类型
    using Ptr = std::shared_ptr<FeatureNode>;

public:
    /// @brief 特征节点映射表类型（key为标识符，value为节点指针）
    using FeatureNodeMap = std::unordered_map<std::string, Ptr>;
    /// @brief 绘制掩码类型
    using DrawMask = std::uint64_t;

public:
    /**
     * @brief 图像信息缓存块
     *
     * @note 存储了特征节点识别出的图像级信息，例如轮廓、角点和方向等。
     */
    struct ImageCache
    {
        /// @brief 轮廓组
        DEFINE_PROPERTY(Contours, public, public, (std::vector<Contour_cptr>));
        /// @brief 角点集
        DEFINE_PROPERTY(Corners, public, public, (std::vector<cv::Point2f>));
        /// @brief 中心位置
        DEFINE_PROPERTY(Center, public, public, (cv::Point2f));
        /// @brief 方向向量
        DEFINE_PROPERTY(Direction, public, public, (cv::Point2f));
        /// @brief 宽度（像素单位）
        DEFINE_PROPERTY(Width, public, public, (float));
        /// @brief 高度（像素单位）
        DEFINE_PROPERTY(Height, public, public, (float));
    };

    /**
     * @brief 位姿信息缓存块
     *
     * @note 存储特征节点对应的位姿数据，例如位姿节点映射和陀螺仪信息。
     */
    struct PoseCache
    {
        /// @brief 位姿节点映射表类型
        using PoseNodeMap = std::unordered_map<std::string, PoseNode>;
        /// @brief 位姿节点映射
        DEFINE_PROPERTY_WITH_INIT(PoseNodes, public, public, (PoseNodeMap), PoseNodeMap{});
        /// @brief 陀螺仪位姿信息
        DEFINE_PROPERTY(GyroData, public, public, (GyroData));
    };

    /// @brief 绘制配置结构体前置声明
    struct DrawConfig;
    /// @brief 绘制配置共享指针
    using DrawConfig_ptr = std::shared_ptr<DrawConfig>;
    /// @brief 常量绘制配置共享指针
    using DrawConfig_cptr = std::shared_ptr<const DrawConfig>;

    /// @brief 子特征节点映射表类型前置声明
    struct ChildFeatureType;

private:
    /// @brief 图像信息缓存
    DEFINE_PROPERTY_WITH_INIT(ImageCache, public, protected, (ImageCache), ImageCache());
    /// @brief 位姿信息缓存
    DEFINE_PROPERTY_WITH_INIT(PoseCache, public, protected, (PoseCache), PoseCache());
    /// @brief 子特征节点映射表
    DEFINE_PROPERTY_WITH_INIT(ChildFeatures, public, protected, (FeatureNodeMap), FeatureNodeMap());
    /// @brief 构建时间戳（单位：tick）
    DEFINE_PROPERTY(Tick, public, public, (int64_t));

public:
    /**
     * @brief 构造函数
     */
    FeatureNode() = default;

    /**
     * @brief 析构函数
     */
    virtual ~FeatureNode() = default;

    /**
     * @brief 访问图像信息缓存
     * @return 图像信息缓存的常量引用
     */
    virtual const ImageCache &imageCache() const noexcept { return this->getImageCache(); }

    /**
     * @brief 访问位姿信息缓存
     * @return 位姿信息缓存的常量引用
     */
    virtual const PoseCache &poseCache() const noexcept { return this->getPoseCache(); }

    /**
     * @brief 获取子特征节点映射
     * @return 子特征节点映射的常量引用
     */
    virtual const FeatureNodeMap &childFeatures() const noexcept { return this->getChildFeatures(); }

    /**
     * @brief 绘制特征节点
     *
     * @param[in,out] image 绘制目标图像
     * @param[in] config 绘制配置指针，默认为nullptr
     *
     * @note 默认实现为空，子类可以重写此函数以实现具体绘制逻辑。
     */
    virtual void drawFeature(cv::Mat &image, const DrawConfig_cptr &config = nullptr) const
    {
        (void)image;  // 避免未使用参数警告
        (void)config; // 避免未使用参数警告
    }

protected:
};

/// @brief FeatureNode 类型共享指针
using FeatureNode_ptr = std::shared_ptr<FeatureNode>;
/// @brief FeatureNode 常量类型共享指针
using FeatureNode_cptr = std::shared_ptr<const FeatureNode>;

#include "feature_node_draw_config.h"        // 绘制配置定义
#include "feature_node_child_feature_type.h" // 子特征节点类型定义
