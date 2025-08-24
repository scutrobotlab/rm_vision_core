/**
 * @file rune_filter_ekf_cv.h
 * @brief 基于 OpenCV 的扩展卡尔曼滤波器实现
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once

#include <opencv2/core.hpp>
#include <optional>

#include "rune_group_filter.h"

/**
 * @brief 基于 OpenCV 的扩展卡尔曼滤波器
 *
 * 用于对神符位姿进行扩展卡尔曼滤波处理。
 */
class RuneFilterEKF_CV : public RuneFilterStrategy
{
public:
    /**
     * @brief 构造函数
     * @param type 滤波器数据类型 (XYZ / YAW_PITCH / ROLL)
     */
    RuneFilterEKF_CV(RuneFilterDataType type);

    virtual ~RuneFilterEKF_CV() {}

    /**
     * @brief 创建滤波器智能指针
     * @param type 滤波器数据类型
     * @return std::shared_ptr<RuneFilterEKF_CV> 滤波器指针
     */
    static std::shared_ptr<RuneFilterEKF_CV> make_filter(RuneFilterDataType type)
    {
        return std::make_shared<RuneFilterEKF_CV>(type);
    }

    /**
     * @brief 核心滤波函数
     * @param input 滤波输入参数
     * @return FilterOutput 滤波输出
     */
    FilterOutput filter(FilterInput input) override;

    /**
     * @brief 获取预测值
     * @return cv::Matx61d 最新预测位姿 [x y z | yaw pitch roll]
     */
    cv::Matx61d getPredict() override;

    /**
     * @brief 判断滤波器是否有效
     * @return true 有效，false 无效
     */
    bool isValid() override;

    /**
     * @brief 获取滤波器数据类型
     * @return RuneFilterDataType 数据类型
     */
    RuneFilterDataType getDataType() override { return _data_type; }

private:
    RuneFilterDataType _data_type = RuneFilterDataType::XYZ; //!< 滤波器数据类型
    bool initialized_ = false;                               //!< 是否已初始化
    int64 last_tick_ = 0;                                    //!< 上次更新时间戳
    double dt_ = 0.0;                                        //!< 时间间隔

    cv::Mat x_; ///< 状态向量 (12x1)
    cv::Mat P_; ///< 协方差矩阵 (12x12)
    cv::Mat Q_; ///< 过程噪声 (12x12)
    cv::Mat R_; ///< 测量噪声 (6x6)
    cv::Mat H_; ///< 测量矩阵 (6x12)

    /**
     * @brief 状态预测
     * @param dt 时间间隔
     */
    void predict(double dt);

    /**
     * @brief 状态更新
     * @param z 测量向量 (6x1) [x y z yaw pitch roll]
     */
    void update(const cv::Mat &z);
};
