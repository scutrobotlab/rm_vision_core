/**
 * @file param_view_manager.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 参数可视化管理器
 * @date 2025-7-15
 */

#pragma once

#include "param_canvas.h"
#include <memory>
#include <string>
#include <unordered_map>

class ParamViewManager
{
public:
    ParamViewManager();
    using Ptr = std::shared_ptr<ParamViewManager>;

    /**
     * @brief 构造接口
     */
    static Ptr create() { return Ptr(new ParamViewManager()); }

    /**
     * @brief 添加单个参数数据到指定画布
     * @param[in] canvas_name 画布名称
     * @param[in] param_key 参数键
     * @param[in] value 参数值
     * @param[in] max_points 最大点数，默认为300
     */
    void addParam(const std::string &canvas_name, const std::string &param_key, float value);

    /**
     * @brief 添加单个参数数据到指定画布
     * @param[in] canvas_name 画布名称
     * @param[in] param_key 参数键
     * @param[in] values 参数值数组
     * @param[in] overwrite 是否覆盖已有数据，默认为false
     */
    void addParam(const std::string &canvas_name, const std::string &param_key, const std::vector<float> &values, bool overwrite = false);

    /**
     * @brief 重置指定画布的数据
     *
     * @param[in] canvas_name 画布名称
     */
    void resetCanvas(const std::string &canvas_name);

    /**
     * @brief 重置所有画布的数据
     */
    void resetAll();

    /**
     * @brief 显示所有画布窗口
     */
    void showAll();

    /**
     * @brief 设置指定画布的尺寸
     * @param[in] canvas_name 画布名称
     * @param[in] width 画布宽度
     * @param[in] height 画布高度
     */
    void setCanvasSize(const std::string &canvas_name, int width, int height);

    /**
     * @brief 设置指定画布的刻度
     * @param[in] canvas_name 画布名称
     * @param[in] x_ticks X轴刻度数量
     * @param[in] y_ticks Y轴刻度数量
     */
    void setCanvasTicks(const std::string &canvas_name, int x_ticks, int y_ticks);

    /**
     * @brief 设置指定画布的最大点数
     * @param[in] canvas_name 画布名称
     * @param[in] max_points 最大点数
     */
    void setCanvasMaxPoints(const std::string &canvas_name, int max_points);

private:
    std::unordered_map<std::string, std::shared_ptr<ParamCanvas>> canvas_map_;
};
