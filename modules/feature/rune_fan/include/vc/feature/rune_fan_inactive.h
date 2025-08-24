/**
 * @file rune_fan_inactive.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 未激活的神符扇叶特征类头文件
 * @date 2025-7-15
 * @details 定义了继承自 RuneFan 的未激活扇叶特征类 RuneFanInactive，
 *          包含箭头轮廓、角点矫正、方向矫正、PNP位姿构造及可视化接口。
 *          提供静态方法用于批量查找、构造和获取末端箭头轮廓。
 */

#pragma once

#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_fan_hump.h"

/**
 * @class RuneFanInactive
 * @brief 未激活的神符扇叶特征类
 * @details 继承自 RuneFan，包含箭头轮廓信息及角点/方向矫正接口。
 *          提供构造、查找、PNP构造及绘制特征接口。
 */
class RuneFanInactive : public RuneFan
{
    using Ptr = std::shared_ptr<RuneFanInactive>; //!< 智能指针类型定义

    DEFINE_PROPERTY(ArrowContours, public, protected, (std::vector<Contour_cptr>)); //!< 箭头轮廓组

public:
    RuneFanInactive() = default;                       //!< 默认构造
    RuneFanInactive(const RuneFanInactive &) = delete; //!< 禁用拷贝构造
    RuneFanInactive(RuneFanInactive &&) = delete;      //!< 禁用移动构造
    virtual ~RuneFanInactive() = default;              //!< 默认析构

    /**
     * @brief 将通用 FeatureNode 智能指针转换为 RuneFanInactive 智能指针
     * @param p_feature 输入 FeatureNode 智能指针
     * @return 转换后的 RuneFanInactive 智能指针
     */
    static inline Ptr cast(FeatureNode_ptr p_feature) { return std::dynamic_pointer_cast<RuneFanInactive>(p_feature); }

    /**
     * @brief 使用轮廓和旋转矩形构造未激活 RuneFan
     * @param[in] hull_contour 凸包轮廓
     * @param[in] arrow_contours 箭头轮廓
     * @param[in] rotated_rect 最小面积矩形
     */
    RuneFanInactive(const Contour_cptr hull_contour, const std::vector<Contour_cptr> &arrow_contours, const cv::RotatedRect &rotated_rect);

    /**
     * @brief 使用四个顶点构造未激活 RuneFan
     * @param[in] top_left 左上顶点
     * @param[in] top_right 右上顶点
     * @param[in] bottom_right 右下顶点
     * @param[in] bottom_left 左下顶点
     */
    RuneFanInactive(const cv::Point2f &top_left, const cv::Point2f &top_right, const cv::Point2f &bottom_right, const cv::Point2f &bottom_left);

    /**
     * @brief 找出所有未激活扇叶
     * @param[out] fans 输出扇叶数组
     * @param[in] contours 所有轮廓
     * @param[in] hierarchy 层级向量
     * @param[in] mask 可跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标
     * @param[in] inactive_targets 未激活的靶心
     */
    static void find(std::vector<FeatureNode_ptr> &fans, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs, const std::vector<FeatureNode_cptr> &inactive_targets);

    /**
     * @brief 未激活 RuneFan 的强制构造接口（角点输入）
     * @param[in] top_left 左上角点
     * @param[in] top_right 右上角点
     * @param[in] bottom_right 右下角点
     * @param[in] bottom_left 左下角点
     * @return 成功返回 RuneFanInactive 智能指针，否则返回 nullptr
     * @note 利用PNP解算获取角点构造
     */
    static std::shared_ptr<RuneFanInactive> make_feature(const cv::Point2f &top_left, const cv::Point2f &top_right, const cv::Point2f &bottom_right, const cv::Point2f &bottom_left);

    /**
     * @brief 未激活扇叶的方向矫正
     * @param[in] fan 待矫正扇叶
     * @param[in] correct_center 矫正中心
     * @return 矫正成功返回 true，否则返回 false
     */
    static bool correctDirection(FeatureNode_ptr &fan, const cv::Point2f &correct_center);

    /**
     * @brief 未激活扇叶的角点矫正
     * @param[in,out] fan 未激活扇叶
     * @return 是否矫正成功
     * @note 矫正后向量 (__corners[2] - __corners[1]) 指向神符中心
     */
    static bool correctCorners(FeatureNode_ptr &fan);

    /**
     * @brief 提取末端箭头轮廓
     * @param[in] fan 待提取扇叶
     * @param[in] target_inactive 未激活的靶心
     * @return 返回末端箭头轮廓，失败则返回 nullptr
     * @note 1. 返回灯臂中距离靶心最远的轮廓，用于强制构造神符中心
     *       2. 调用此函数会重新构造 fan，需在 fan 的 correct 函数前调用
     */
    static Contour_cptr getEndArrowContour(FeatureNode_ptr &fan, const std::vector<FeatureNode_cptr> &target_inactive);

    /**
     * @brief 获取角点在图像坐标系和特征坐标系下的坐标
     * @return [0] 图像坐标系 [1] 特征坐标系 [2] 各点权重
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> override;

    /**
     * @brief 绘制特征
     * @param image 要绘制的图像
     * @param config 绘制配置，可选
     * @note 默认实现为空，子类可重载实现具体绘制逻辑
     */
    virtual void drawFeature(cv::Mat &image, const DrawConfig_cptr &config = nullptr) const override;

protected:
    /**
     * @brief 未激活 RuneFan 的轮廓构造接口
     * @param[in] contours 轮廓组
     * @return 成功返回 RuneFanInactive 智能指针，否则返回 nullptr
     */
    static Ptr make_feature(const std::vector<Contour_cptr> &contours);
};

using RuneFanInactive_ptr = std::shared_ptr<RuneFanInactive>;        //!< 可修改智能指针
using RuneFanInactive_cptr = std::shared_ptr<const RuneFanInactive>; //!< 只读智能指针
