/**
 * @file debug_tools.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 调试工具类定义文件
 * @date 2025-8-20
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <mutex>
#include <string>

class ParamViewManager; //!< 参数视图管理器

/**
 * @brief 调试工具类
 *
 * @note 提供全局单例访问，支持缓存图像、绘制调试信息以及显示窗口。
 *       适用于逐帧视觉处理程序的调试可视化。
 */
class DebugTools
{
    /// @brief 智能指针类型
    using Ptr = std::shared_ptr<DebugTools>;

public:
    /**
     * @brief 构造函数
     */
    DebugTools() = default;

    /**
     * @brief 获取单例实例
     * @return DebugTools 单例智能指针
     */
    static Ptr get();

    /**
     * @brief 设置缓存图像
     * @param[in] img 待缓存的图像
     *
     * @note 通常在处理开始时调用，用于后续绘制调试信息。
     */
    void setImage(const cv::Mat &img);

    /**
     * @brief 获取缓存图像
     * @return cv::Mat 返回当前缓存图像的副本
     *
     * @note 用于在调试或绘制过程中访问当前图像。
     */
    cv::Mat getImage();

    /**
     * @brief 获取参数视图管理器
     * @return ParamViewManager_ptr 返回参数视图管理器的指针
     */
    std::shared_ptr<ParamViewManager> getPVM();

    /**
     * @brief 在指定窗口显示缓存图像
     * @param[in] winName 窗口名称，默认为 "Debug"
     */
    void show(const std::string &winName = "Debug");

    // 禁止复制和赋值
    DebugTools(const DebugTools &) = delete;
    DebugTools &operator=(const DebugTools &) = delete;

private:
    /**
     * @brief 初始化显示窗口
     * @param[in] winName 窗口名称，默认为 "Debug"
     */
    void initWindow(const std::string &winName = "Debug");

    cv::Mat cache_;                   //!< 缓存图像
    std::mutex mtx_;                  //!< 互斥锁，用于线程安全
    bool window_initialized_ = false; //!< 窗口初始化状态
    std::shared_ptr<ParamViewManager> pvm_ = nullptr; //!< 参数视图管理器
};

/// @brief DebugTools 智能指针类型
using DebugTools_ptr = std::shared_ptr<DebugTools>;
