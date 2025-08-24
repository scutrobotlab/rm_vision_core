/**
 * @file rune_filter_fusion.h
 * @brief 神符序列组融合滤波器头文件
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include "rune_group_filter.h"
#include "rune_filter_type.h"

/**
 * @brief 神符序列组融合滤波器
 *
 * 通过组合多个子滤波器实现对神符位姿的融合滤波。
 */
class RuneFilterFusion : public RuneFilterStrategy
{
public:
    RuneFilterFusion() = default;

    /**
     * @brief 构建融合滤波器实例
     * @return std::shared_ptr<RuneFilterStrategy> 滤波器智能指针
     */
    static std::shared_ptr<RuneFilterStrategy> make_filter();

    /**
     * @brief 滤波主函数
     * @param input 滤波输入参数
     * @return FilterOutput 滤波输出结果
     */
    virtual FilterOutput filter(FilterInput input) override;

    /**
     * @brief 获取最新帧的预测值
     * @return cv::Matx61d 滤波器预测位姿 [x y z | yaw pitch roll]
     */
    cv::Matx61d getPredict() override;

    /**
     * @brief 判断滤波器是否有效
     * @return true 有效，false 无效
     */
    bool isValid() override;

    /**
     * @brief 获取滤波数据类型
     * @return RuneFilterDataType 组合的滤波数据类型标志
     */
    RuneFilterDataType getDataType() override
    {
        return RuneFilterDataType::XYZ | RuneFilterDataType::YAW_PITCH | RuneFilterDataType::ROLL;
    }

protected:
    /**
     * @brief 初始化融合滤波器
     * @param[in] first_pos 第一次滤波的位置 [x y z | yaw pitch roll]
     * @param[in] tick 时间戳
     */
    void initFilter(const cv::Matx61d &first_pos, const int64 tick);

protected:
    bool _is_init_filter = false;         //!< 滤波器是否已初始化
    std::vector<RuneFilter_ptr> _filters; //!< 子滤波器集合
};

//! 智能指针类型定义
using RuneFilterFusion_ptr = std::shared_ptr<RuneFilterFusion>;
