/**
 * @file rune_target_inactive.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 神符未激活靶心类定义
 * @date 2025-08-24
 *
 * @details
 * 定义继承自 RuneTarget 的未激活靶心类 RuneTargetInactive。
 * 提供靶心查找、矫正、PNP 角点获取、缺口处理及特征绘制接口。
 * 同时定义智能指针类型。
 */

#pragma once

#include "rune_target.h"

//! 神符靶心特征的缺口
struct RuneTargetGap
{
    cv::Point2f left_corner;  //!< 左角点
    cv::Point2f right_corner; //!< 右角点
    cv::Point2f center;       //!< 中心点
    bool is_valid = true;     //!< 是否有效
};

/**
 * @class RuneTargetInactive
 * @brief 神符未激活靶心特征类
 *
 * 继承自 RuneTarget，用于表示未激活状态的神符靶心。
 * 包含缺口信息、方向、角点修正等功能。
 */
class RuneTargetInactive : public RuneTarget
{
    using Ptr = std::shared_ptr<RuneTargetInactive>; //!< 智能指针类型

public:
    RuneTargetInactive() = default;
    RuneTargetInactive(const RuneTargetInactive &) = delete;
    RuneTargetInactive(RuneTargetInactive &&) = delete;
    virtual ~RuneTargetInactive() = default;

    /**
     * @brief 构造未激活靶心
     * @param[in] contour 靶心轮廓
     * @param[in] corners 靶心角点
     * @param[in] gaps 靶心缺口集合
     */
    RuneTargetInactive(const Contour_cptr contour, const std::vector<cv::Point2f> corners,std::vector<RuneTargetGap> gaps);
    
    /**
     * @brief 构造函数（中心点与角点）
     * @param center 靶心中心点
     * @param corners 图像坐标系角点集合
     */
    RuneTargetInactive(const cv::Point2f center, const std::vector<cv::Point2f> corners);

    /**
     * @brief 动态类型转换
     * @param[in] p_feature FeatureNode 智能指针
     * @return 转换后的 RuneTargetInactive 智能指针
     */
    static inline Ptr cast(FeatureNode_ptr p_feature) {return std::dynamic_pointer_cast<RuneTargetInactive>(p_feature);}

    /**
     * @brief 查找所有未激活靶心
     * @param[out] targets 返回找到的靶心集合
     * @param[in] contours 输入轮廓集合
     * @param[in] hierarchy 轮廓层级信息
     * @param[in] mask 可跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 输出使用过的轮廓下标集合
     */
    static void find(std::vector<FeatureNode_ptr> &targets, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 未激活靶心的矫正
     * @param[in] rotate_center 旋转中心
     * @return 矫正是否成功
     */
    bool correct(const cv::Point2f &rotate_center);

    /**
     * @brief 通过神符位姿 PnP 解算结果构造 RuneTargetInactive
     * @param[in] target_to_cam 靶心相对于相机的位姿
     * @return 构造的 RuneTargetInactive 智能指针，失败返回 nullptr
     */
    static std::shared_ptr<RuneTargetInactive> make_feature(const PoseNode &target_to_cam);

protected:
    /**
     * @brief 未激活 RuneTarget 的构造接口
     * @param[in] contours 所有轮廓
     * @param[in] hierarchy 所有层级向量
     * @param[in] idx 最外层轮廓下标
     * @param[out] used_contour_idxs 使用过的轮廓下标
     * @return 构造的 RuneTargetInactive 智能指针
     */
    static Ptr make_feature(const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, size_t idx, std::unordered_set<size_t> &used_contour_idxs);

    /**
     * @brief 通过神符位姿 PnP 构造 RuneTarget
     * @param[in] target_to_cam 靶心相对于相机的位姿
     * @param[in] is_active 是否激活
     * @return 构造的 RuneTargetInactive 智能指针
     */
    static Ptr make_feature(PoseNode &target_to_cam, bool is_active);

    /**
     * @brief 未激活靶心方向修正
     * @return 修正是否成功
     */
    bool correctDirection();

    /**
     * @brief 未激活扇叶角点修正
     * @param[in] rune_center 神符中心
     * @return 修正是否成功
     */
    bool correctCorners(const cv::Point2f &rune_center);

    /**
     * @brief 未激活靶心缺口排序
     * @param[in] rune_center 神符中心
     * @return 排序是否成功
     */
    bool sortedGap(const cv::Point2f &rune_center);

    /**
     * @brief 设置缺口角点
     * @param[in] rune_center 神符中心
     * @return 计算是否成功
     */
    bool calcGapCorners(const cv::Point2f &rune_center);

    /**
     * @brief 获取所有 PnP 角点
     * @return 元组，包含图像坐标、特征坐标和权重
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> override;

    /**
     * @brief 绘制特征节点
     * @param[in,out] image 要绘制的图像
     * @param[in] config 绘制配置，可为空
     */
    virtual void drawFeature(cv::Mat &image, const DrawConfig_cptr &config = nullptr) const override;

protected:
    DEFINE_PROPERTY(Direction, public, protected, (cv::Point2f));                             //!< 方向
    DEFINE_PROPERTY(Gaps, public, protected, (std::vector<RuneTargetGap>));                   //!< 缺口（未排序），排序顺序：左上、右上、右下、左下
    DEFINE_PROPERTY(LeftTopGap, public, protected, (RuneTargetGap *));                        //!< 左上缺口
    DEFINE_PROPERTY(RightTopGap, public, protected, (RuneTargetGap *));                       //!< 右上缺口
    DEFINE_PROPERTY(RightBottomGap, public, protected, (RuneTargetGap *));                    //!< 右下缺口
    DEFINE_PROPERTY(LeftBottomGap, public, protected, (RuneTargetGap *));                     //!< 左下缺口
    DEFINE_PROPERTY_WITH_INIT(GapSortedFlag, public, protected, (bool), false);               //!< 缺口是否已排序
    DEFINE_PROPERTY_WITH_INIT(GapCorners, public, protected, (std::vector<cv::Point2f>), {}); //!< 缺口角点占位

    inline static const cv::Point2f VACANCY_POINT = cv::Point2f(-1, -1); //!< 默认占位点
};

using RuneTargetInactive_ptr = std::shared_ptr<RuneTargetInactive>;        //!< 可修改的智能指针
using RuneTargetInactive_cptr = std::shared_ptr<const RuneTargetInactive>; //!< 只读智能指针
