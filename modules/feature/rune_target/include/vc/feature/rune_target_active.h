/**
 * @file rune_target_active.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 神符已激活靶心类定义
 * @date 2025-08-24
 *
 * @details
 * 定义继承自 RuneTarget 的已激活靶心类 RuneTargetActive。
 * 提供靶心查找、PNP 角点获取、特征绘制接口及智能指针类型定义。
 */

#pragma once

#include "rune_target.h"

/**
 * @class RuneTargetActive
 * @brief 神符已激活靶心类
 *
 * 继承自 RuneTarget，用于表示已激活的靶心。
 * 提供靶心查找、PNP 角点获取及绘制接口。
 */
class RuneTargetActive : public RuneTarget
{
    using Ptr = std::shared_ptr<RuneTargetActive>; //!< 智能指针类型

public:
    RuneTargetActive() = default;
    RuneTargetActive(const RuneTargetActive &) = delete;
    RuneTargetActive(RuneTargetActive &&) = delete;
    virtual ~RuneTargetActive() = default;

    /**
     * @brief 构造函数（轮廓与角点）
     * @param contour 智能轮廓对象
     * @param corners 图像坐标系角点集合
     */
    RuneTargetActive(const Contour_cptr contour, const std::vector<cv::Point2f> corners);

    /**
     * @brief 构造函数（中心点与角点）
     * @param center 靶心中心点
     * @param corners 图像坐标系角点集合
     */
    RuneTargetActive(const cv::Point2f center, const std::vector<cv::Point2f> corners);

    /**
     * @brief 动态类型转换
     * @param p_feature FeatureNode 智能指针
     * @return 转换后的 RuneTargetActive 智能指针
     */
    static inline Ptr cast(FeatureNode_ptr p_feature) { return std::dynamic_pointer_cast<RuneTargetActive>(p_feature); }

    /**
     * @brief 查找所有已激活靶心
     * @param[out] targets 返回找到的靶心集合
     * @param[in] contours 输入轮廓集合
     * @param[in] hierarchy 层级向量
     * @param[in] mask 可跳过的轮廓下标集合
     * @param[out] used_contour_idxs 已使用轮廓下标集合
     */
    static void find(std::vector<FeatureNode_ptr> &targets, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 绘制特征节点
     * @param[in,out] image 绘制图像
     * @param[in] config 绘制配置，可选
     * @note 默认实现为空，子类可重载实现具体绘制逻辑
     */
    virtual void drawFeature(cv::Mat &image, const DrawConfig_cptr &config = nullptr) const override;

    /**
     * @brief 通过 PNP 位姿构造 RuneTargetActive
     * @param[in] target_to_cam 靶心相机位姿
     * @return 成功返回智能指针，否则返回 nullptr
     */
    static std::shared_ptr<RuneTargetActive> make_feature(const PoseNode &target_to_cam);

private:
    /**
     * @brief 内部构造接口（轮廓）
     * @param[in] contours 所有轮廓集合
     * @param[in] hierarchy 层级向量
     * @param[in] idx 最外层轮廓下标
     * @param[out] used_contour_idxs 已使用轮廓下标集合
     * @return 构造的 RuneTargetActive 智能指针
     */
    static Ptr make_feature(const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, size_t idx, std::unordered_set<size_t> &used_contour_idxs);

    /**
     * @brief 获取 PnP 角点
     * @return std::tuple 图像坐标、特征坐标、角点权重
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> override;
};

//! 智能指针类型定义
using RuneTargetActive_ptr = std::shared_ptr<RuneTargetActive>;        //!< 可修改指针
using RuneTargetActive_cptr = std::shared_ptr<const RuneTargetActive>; //!< 只读指针
