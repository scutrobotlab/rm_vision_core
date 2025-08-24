/**
 * @file param_canvas.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 参数可视化画布
 * @date 2025-7-15
 */

#pragma once
#include <deque>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief 用于在 OpenCV 窗口中显示多个参数的动态折线图
 */
class ParamCanvas
{
public:
    ParamCanvas(const std::string &name, int width = 800, int height = 600, int max_points = 300);

    void setCanvasSize(int width, int height);
    void setTicks(int x_ticks, int y_ticks);
    void setMaxPoints(int max_points);
    void setSmoothing(float alpha); // 设置 min/max 平滑衰减率

    void addParam(const std::string &key, float value);
    /**
     * @brief 添加一组参数值到指定键
     * 
     * @param key 参数键
     * @param values 参数值数组
     * @param overwrite 是否覆盖
     */
    void addParam(const std::string &key, const std::vector<float> &values, bool overwrite = false);
    void markX(int x_index, const std::string &label = "", const cv::Scalar &color = {255, 255, 255});
    void markY(float y_value, const std::string &label = "", const cv::Scalar &color = {255, 255, 255});
    void clearMarks();
    void reset();
    void show();

private:
    using ParamQueue = std::deque<float>;

    void windowInit();
    // 参数对齐
    void alignParams();
    void drawAxes(cv::Mat &img, float min_y, float max_y);
    void drawParams(cv::Mat &img, float min_y, float max_y);
    void updateDisplayRange(); // 更新平滑后的 min/max 值

    std::string winname_;
    int img_col_, img_row_;
    int margin_left_, margin_right_, margin_top_, margin_bottom_;
    int num_x_ticks_, num_y_ticks_;
    int max_points_per_param_;
    bool window_init_flag_;
    int window_display_width_ = -1;  //!< 窗口显示宽度（-1 表示不控制）
    int window_display_height_ = -1; //!< 窗口显示高度（-1 表示不控制）
    int window_pos_x_ = -1;          //!< 窗口左上角 X 坐标（-1 表示不控制）
    int window_pos_y_ = -1;          //!< 窗口左上角 Y 坐标（-1 表示不控制）

    float smooth_alpha_; // 平滑因子
    float smooth_min_y_, smooth_max_y_;

    cv::Mat img_;
    std::unordered_map<std::string, ParamQueue> param_map_;
    std::vector<cv::Scalar> colors_;
    std::vector<std::tuple<int, std::string, cv::Scalar>> x_marks_;
    std::vector<std::tuple<float, std::string, cv::Scalar>> y_marks_;

};
