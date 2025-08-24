/**
 * @file rune_fan.h
 * @brief 神符扇叶特征头文件
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once

#include "vc/feature/feature_node.h"

/**
 * @brief 神符扇叶类
 *
 * 继承自 FeatureNode，用于表示神符上的扇叶特征。
 * 提供查找已激活、未激活和残缺扇叶的静态接口，以及通过位姿构造扇叶特征的接口。
 */
class RuneFan : public FeatureNode
{
    using Ptr = std::shared_ptr<RuneFan>;

    //! 激活标志位
    DEFINE_PROPERTY(ActiveFlag, public, protected, (bool));
    //! 外接矩形
    DEFINE_PROPERTY(RotatedRect, public, protected, (cv::RotatedRect));
    //! 误差量
    DEFINE_PROPERTY(Error, public, protected, (float));

public:
    RuneFan() = default;
    RuneFan(const RuneFan &) = delete;
    RuneFan(RuneFan &&) = delete;
    virtual ~RuneFan() = default;

    /**
     * @brief 将 FeatureNode 指针转换为 RuneFan 指针
     * @param p_feature 输入 FeatureNode 指针
     * @return 转换后的 RuneFan 指针
     */
    static inline std::shared_ptr<RuneFan> cast(FeatureNode_ptr p_feature) { return std::dynamic_pointer_cast<RuneFan>(p_feature); }

    /**
     * @brief 将 const FeatureNode 指针转换为 const RuneFan 指针
     * @param p_feature 输入 const FeatureNode 指针
     * @return 转换后的 const RuneFan 指针
     */
    static inline const std::shared_ptr<const RuneFan> cast(FeatureNode_cptr p_feature) { return std::dynamic_pointer_cast<const RuneFan>(p_feature); }

    /**
     * @brief 找到所有未激活扇叶
     * @param[out] fans 返回找到的未激活扇叶
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     */
    static void find_active_fans(std::vector<FeatureNode_ptr> &fans, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 找到所有激活扇叶
     * @param[out] fans 返回找到的激活扇叶
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     * @param[in] inactive_targets 未激活的靶心特征
     */
    static void find_inactive_fans(std::vector<FeatureNode_ptr> &fans, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs, const std::vector<FeatureNode_cptr> &inactive_targets);

    /**
     * @brief 找到所有残缺的已激活扇叶
     * @param[out] fans 返回找到的残缺的已激活扇叶
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[in] rotate_center 旋转中心
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     */
    static void find_incomplete_active_fans(std::vector<FeatureNode_ptr> &fans, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, const cv::Point2f &rotate_center, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 构造接口(通过位姿信息)
     * @param fan_to_cam 扇叶到相机的变换
     * @param is_active 是否激活
     * @return 若构造成功则返回指针，否则返回 nullptr
     */
    static auto make_feature(const PoseNode &fan_to_cam, bool is_active) -> std::shared_ptr<RuneFan>;

    /**
     * @brief 获取角点在图像坐标系和特征坐标系下的坐标
     * @return [0] 图像坐标系 [1] 特征坐标系 [2] 各个点的权重
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>;

    /**
     * @brief 获取角点在图像坐标系和旋转中心坐标系下的坐标
     * @return [0] 图像坐标系 [1] 旋转中心坐标系 [2] 各个点的权重
     */
    auto getRelativePnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>;

protected:
};
using RuneFan_ptr = std::shared_ptr<RuneFan>;
using RuneFan_cptr = std::shared_ptr<const RuneFan>;
