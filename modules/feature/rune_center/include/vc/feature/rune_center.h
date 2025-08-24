/**
 * @file rune_center.h
 * @brief 神符中心特征头文件
 * @author zhaoxi (535394140@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include "vc/feature/feature_node.h"

/**
 * @brief 神符中心特征
 *
 * 继承自 FeatureNode，用于表示神符的中心特征。
 * 提供查找中心、构造特征以及获取 PnP 点的接口。
 */
class RuneCenter : public FeatureNode
{
private:
public:
    RuneCenter() = delete;
    RuneCenter(const RuneCenter &) = delete;
    RuneCenter(RuneCenter &&) = delete;

    /**
     * @brief 构造函数（轮廓与旋转矩形）
     * @param contour 智能轮廓对象
     * @param rotated_rect 轮廓的旋转矩形
     */
    RuneCenter(const Contour_cptr &contour, cv::RotatedRect &rotated_rect);

    /**
     * @brief 构造函数（中心点）
     * @param center 中心点
     */
    RuneCenter(const cv::Point2f &center);

    /**
     * @brief 找出所有中心
     * @param[out] centers 输出的神符中心向量
     * @param[in] contours 所有轮廓
     * @param[in] hierarchy 所有的等级向量
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标
     */
    static void find(std::vector<FeatureNode_ptr> &centers, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 使用特征中心点构造 RuneCenter 的构造接口
     * @param[in] center 特征中心点
     * @param[in] force 是否为强制构造
     * @return 如果成功，返回 RuneCenter 的共享指针，否则返回 nullptr
     */
    static std::shared_ptr<RuneCenter> make_feature(const cv::Point2f &center, bool force = false);

    /**
     * @brief 使用轮廓和层次结构构造 RuneCenter 的构造接口
     * @param[in] contour 轮廓
     * @param[in] sub_contours 子轮廓
     * @return 如果成功，返回 RuneCenter 的共享指针，否则返回 nullptr
     */
    static std::shared_ptr<RuneCenter> make_feature(const Contour_cptr &contour, const std::vector<Contour_cptr> &sub_contours);

    /**
     * @brief 使用神符中心的位姿构造 RuneCenter
     * @param[in] center_to_cam 神符中心相对于相机的位姿解算结果
     */
    static std::shared_ptr<RuneCenter> make_feature(const PoseNode &center_to_cam);

    /**
     * @brief 获取角点在图像坐标系和特征坐标系下的坐标
     * @return [0] 图像坐标系 [1] 特征坐标系 [2] 各个点的权重
     */
    auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>;

    /**
     * @brief 获取相对旋转中心坐标系下的坐标
     * @return [0] 图像坐标系 [1] 旋转中心坐标系 [2] 各个点的权重
     */
    auto getRelativePnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>;

    /**
     * @brief 动态类型转换
     * @param[in] p_feature feature_ptr 抽象指针
     * @return 派生对象指针
     */
    static inline std::shared_ptr<RuneCenter> cast(FeatureNode_ptr p_feature) { return std::dynamic_pointer_cast<RuneCenter>(p_feature); }
    static inline const std::shared_ptr<const RuneCenter> cast(FeatureNode_cptr p_feature) { return std::dynamic_pointer_cast<const RuneCenter>(p_feature); }

    /**
     * @brief 绘制特征
     * @param image 要绘制的图像
     * @param config 绘制配置
     * @note 默认实现为空，子类可重载实现具体绘制逻辑
     */
    virtual void drawFeature(cv::Mat &image, const DrawConfig_cptr &config = nullptr) const override;
};

//! 神符中心特征共享指针
using RuneCenter_ptr = std::shared_ptr<RuneCenter>;
using RuneCenter_cptr = std::shared_ptr<const RuneCenter>;
