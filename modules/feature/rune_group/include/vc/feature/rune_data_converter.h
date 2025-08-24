/**
 * @file rune_data_converter.h
 * @brief 数据转化器类头文件
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include "vc/feature/rune_group.h"

/**
 * @brief 数据转化器
 *
 * 提供滤波形式数据与平移向量/旋转向量之间的相互转换。
 */
class DataConverter
{
public:
    DataConverter() = default;

    /**
     * @brief 获取 DataConverter 智能指针实例
     * @return std::shared_ptr<DataConverter> 转化器实例
     */
    static std::shared_ptr<DataConverter> make_converter();

    /**
     * @brief 将可滤波形式的数据转化为平移向量和旋转向量
     *
     * @param[in] filter_pos 可滤波形式位姿 [x y z | yaw pitch roll]
     * @return std::tuple<cv::Matx31f, cv::Matx31f> 平移向量 tvec 和旋转向量 rvec
     */
    static std::tuple<cv::Matx31f, cv::Matx31f> toTvecAndRvec(const cv::Matx61f &filter_pos);

    /**
     * @brief 将平移向量和旋转向量转化为可滤波形式
     *
     * @param[in] tvec 平移向量
     * @param[in] rvec 旋转向量
     * @param[out] is_gimbal_lock 是否发生万向节锁死
     * @param[in] is_reset 是否重置连续性处理
     * @return cv::Matx61f 可滤波形式位姿 [x y z | yaw pitch roll]
     */
    cv::Matx61f toFilterForm(const cv::Vec3f &tvec, const cv::Vec3f &rvec, bool &is_gimbal_lock, bool is_reset = false);

    /**
     * @brief 欧拉角转旋转向量
     *
     * @param[in] yaw 欧拉角 yaw（度）
     * @param[in] pitch 欧拉角 pitch（度）
     * @param[in] roll 欧拉角 roll（度）
     * @return cv::Matx31f 对应旋转向量
     */
    static cv::Matx31f euler2Rvec(float yaw, float pitch, float roll);

    /**
     * @brief 旋转向量转欧拉角
     *
     * @param[in] rvec 旋转向量
     * @return std::tuple<float, float, float> 欧拉角 (yaw, pitch, roll)
     */
    static std::tuple<float, float, float> rvec2Euler(const cv::Vec3f &rvec);

    /**
     * @brief 将平移向量和旋转向量转换为滤波器所需位姿形式
     *
     * @param[in] tvec 平移向量
     * @param[in] rvec 旋转向量
     * @param[out] is_gimbal_lock 是否发生万向节锁死
     * @return cv::Matx61f 可滤波形式位姿 [x y z | yaw pitch roll]
     */
    static cv::Matx61f cvtPos(const cv::Vec3f &tvec, const cv::Vec3f &rvec, bool &is_gimbal_lock);

private:
    float _last_yaw = 0;  //!< 上一帧 yaw，用于连续性处理
    float _last_roll = 0; //!< 上一帧 roll，用于连续性处理
};

//! 智能指针类型
using DataConverter_ptr = std::shared_ptr<DataConverter>;
