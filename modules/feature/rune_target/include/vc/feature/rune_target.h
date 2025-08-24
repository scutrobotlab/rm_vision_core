/**
 * @file rune_target.h
 * @author 张华铨 (1325694319@qq.com)
 * @version 0.1
 * @date 2025-08-20
 *
 * @details
 * 定义神符靶心特征类 RuneTarget 及其智能指针类型。
 * 提供构造函数、静态查找激活/未激活靶心接口、PNP角点获取接口。
 */

#pragma once

#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/feature/feature_node.h"
#include "vc/math/pose_node.hpp"
#include <unordered_map>
#include <unordered_set>

/**
 * @class RuneTarget
 * @brief 神符靶心特征类
 *
 * 继承自 FeatureNode，用于表示单个神符靶心的特征信息。
 * 包含轮廓数据、角点信息及激活状态标志。
 * 提供静态函数用于查找激活或未激活的靶心，并提供 PnP 坐标获取接口。
 */
class RuneTarget : public FeatureNode
{
    using Ptr = std::shared_ptr<RuneTarget>; //!< RuneTarget 智能指针类型

    //! 激活标志位
    DEFINE_PROPERTY(ActiveFlag, public, protected, (bool));

public:
    RuneTarget() = default;
    RuneTarget(const RuneTarget &) = delete;
    RuneTarget(RuneTarget &&) = delete;
    virtual ~RuneTarget() = default;

    /**
     * @brief 构造函数
     * @param contour 智能轮廓对象（std::shared_ptr<const ContourWrapper<int>>），用于标定靶心形状
     * @param corners 角点集合（图像坐标系，cv::Point2f）
     */
    RuneTarget(const Contour_cptr contour, const std::vector<cv::Point2f> corners);

    /**
     * @brief 类型转换函数，将 FeatureNode 智能指针转换为 RuneTarget 智能指针
     * @param p_feature 输入的 FeatureNode 智能指针（std::shared_ptr<FeatureNode>）
     * @return 转换后的 RuneTarget 智能指针（std::shared_ptr<RuneTarget>），可能为 nullptr
     */
    static inline std::shared_ptr<RuneTarget> cast(FeatureNode_ptr p_feature) { return std::dynamic_pointer_cast<RuneTarget>(p_feature); }

    /**
     * @brief 类型转换函数，将 const FeatureNode 智能指针转换为 const RuneTarget 智能指针
     * @param p_feature 输入的 const FeatureNode 智能指针（std::shared_ptr<const FeatureNode>）
     * @return 转换后的 const RuneTarget 智能指针（std::shared_ptr<const RuneTarget>），可能为 nullptr
     */
    static inline const std::shared_ptr<const RuneTarget> cast(FeatureNode_cptr p_feature) { return std::dynamic_pointer_cast<const RuneTarget>(p_feature); }

    /**
     * @brief 查找所有未激活靶心
     *
     * 从轮廓集合中筛选出所有未激活的靶心特征，并构造 FeatureNode 对象。
     *
     * @param[out] targets 返回找到的未激活靶心集合（std::vector<std::shared_ptr<FeatureNode>>）
     * @param[in] contours 输入轮廓集合（std::vector<std::shared_ptr<const ContourWrapper<int>>>）
     * @param[in] hierarchy 轮廓层级信息（OpenCV findContours 输出 std::vector<cv::Vec4i>）
     * @param[in] mask 可跳过的轮廓下标集合（std::unordered_set<size_t>）
     * @param[out] used_contour_idxs 输出使用过的轮廓下标集合（std::unordered_map<std::shared_ptr<const FeatureNode>, std::unordered_set<size_t>>）
     */
    static void find_inactive_targets(std::vector<FeatureNode_ptr> &targets, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 查找所有激活靶心
     *
     * 从轮廓集合中筛选出所有激活的靶心特征，并构造 FeatureNode 对象。
     *
     * @param[out] targets 返回找到的激活靶心集合（std::vector<std::shared_ptr<FeatureNode>>）
     * @param[in] contours 输入轮廓集合（std::vector<std::shared_ptr<const ContourWrapper<int>>>）
     * @param[in] hierarchy 轮廓层级信息（OpenCV findContours 输出 std::vector<cv::Vec4i>）
     * @param[in] mask 可跳过的轮廓下标集合（std::unordered_set<size_t>）
     * @param[out] used_contour_idxs 输出使用过的轮廓下标集合（std::unordered_map<std::shared_ptr<const FeatureNode>, std::unordered_set<size_t>>）
     */
    static void find_active_targets(std::vector<FeatureNode_ptr> &targets, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 获取角点在图像坐标系和特征坐标系下的坐标
     *
     * 返回用于 PnP 求解的角点集合，包括权重信息。
     *
     * @return std::tuple
     * - [0] 图像坐标系下角点集合（std::vector<cv::Point2f>）
     * - [1] 特征坐标系下角点集合（std::vector<cv::Point3f>）
     * - [2] 各个角点的权重（std::vector<float>）
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>;

    /**
     * @brief 获取角点在图像坐标系和旋转中心坐标系下的坐标
     *
     * 返回旋转中心坐标系下的角点集合，可用于相对运动分析。
     *
     * @return std::tuple
     * - [0] 图像坐标系下角点集合（std::vector<cv::Point2f>）
     * - [1] 旋转中心坐标系下角点集合（std::vector<cv::Point3f>）
     * - [2] 各个角点的权重（std::vector<float>）
     */
    auto getRelativePnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>;
};

//! 神符靶心特征智能指针类型
using RuneTarget_ptr = std::shared_ptr<RuneTarget>;        //!< 可修改的 RuneTarget 智能指针
using RuneTarget_cptr = std::shared_ptr<const RuneTarget>; //!< 只读 RuneTarget 智能指针
