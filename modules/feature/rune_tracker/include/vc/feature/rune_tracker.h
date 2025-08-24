/**
 * @file rune_tracker.h
 * @brief 神符时间序列追踪器头文件
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include "vc/feature/tracking_feature_node.h"

//! 神符时间序列追踪器
class RuneTracker : public TrackingFeatureNode
{
    using Ptr = std::shared_ptr<RuneTracker>;

    //! 掉帧数
    DEFINE_PROPERTY_WITH_INIT(DropFrameCount, public, protected, (int), 0);

public:
    RuneTracker() = default;
    RuneTracker(const RuneTracker &) = delete;
    RuneTracker(RuneTracker &&) = delete;
    virtual ~RuneTracker() = default;

    /**
     * @brief 构建 RuneTracker
     *
     * @return 新建的 RuneTracker 智能指针
     */
    static Ptr make_feature() { return std::make_shared<RuneTracker>(); };

    /**
     * @brief 动态类型转换
     *
     * @param[in] p_tracker 抽象 FeatureNode 指针
     * @return 转换后的 RuneTracker 智能指针
     */
    static inline Ptr cast(FeatureNode_ptr p_tracker)
    {
        return std::dynamic_pointer_cast<RuneTracker>(p_tracker);
    }

    /**
     * @brief 更新时间序列
     *
     * @param[in] p_rune 神符共享指针
     * @param[in] tick 当前时间戳
     * @param[in] gyro_data 云台陀螺仪数据
     */
    void update(FeatureNode_ptr p_rune, int64 tick, const GyroData &gyro_data);

    /**
     * @brief 更新可见性
     *
     * @param[in] is_visible 是否可见
     */
    void updateVisible(bool is_visible);

    /**
     * @brief 绘制特征节点
     *
     * @param[in,out] image 绘制目标图像
     * @param[in] config 绘制配置指针，默认为nullptr
     *
     * @note 默认实现为空，子类可以重写此函数以实现具体绘制逻辑。
     */
    virtual void drawFeature(cv::Mat &image, const DrawConfig_cptr &config = nullptr) const override;
private:
    /**
     * @brief 从神符组合体更新内部数据
     *
     * @param[in] p_combo 神符组合体共享指针
     */
    void updateFromRune(FeatureNode_ptr p_combo);
};

//! 神符追踪器智能指针类型
using RuneTracker_ptr = std::shared_ptr<RuneTracker>;
using RuneTracker_cptr = std::shared_ptr<const RuneTracker>;
