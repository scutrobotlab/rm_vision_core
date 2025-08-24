/**
 * @file window_auto_layout.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief OpenCV 窗口自动布局器
 * @date 2025-7-15
 */

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

//! 窗口自动布局类(单例模式)
class WindowAutoLayout
{
public:
    using Ptr = std::shared_ptr<WindowAutoLayout>; //!< 智能指针类型定义
    static Ptr get();                              //!< 获取单例实例
    /**
     * @brief 构造函数
     * @param screen 屏幕尺寸（逻辑管理范围），默认 1920x1080
     * @param minWin 每个窗口的最小尺寸阈值，默认 320x240
     */
    explicit WindowAutoLayout(cv::Size screen = {1920, 1080}, cv::Size minWin = {320, 240});

    /** @brief 设置屏幕尺寸（用于布局的可用区域） */
    void setScreenSize(const cv::Size &s);

    /** @brief 设置窗口最小阈值（小于该尺寸则转为覆盖模式） */
    void setMinWindowSize(const cv::Size &s);

    /**
     * @brief 动态添加窗口（若不存在则创建，随后触发整体重排）
     * @param name  唯一窗口名
     * @param desired 期望尺寸（仅作为参考，最终由布局决定）
     * @param flags  cv::namedWindow 的 flags（默认 WINDOW_NORMAL ）
     */
    void addWindow(const std::string &name, cv::Size desired = {640, 480}, int flags = cv::WINDOW_NORMAL);

    /** @brief 移除并销毁窗口 */
    void removeWindow(const std::string &name);

    /** @brief 查询窗口是否存在 */
    bool hasWindow(const std::string &name) const;

    /** @brief 强制重新布局（例如外部改变了屏幕尺寸） */
    void relayout();

    /**
     * @brief 设置内部动画
     * @param enabled 是否启用
     * @param duration_ms 单次过渡时长（毫秒）
     * @param steps 插值帧数（越大越顺滑）
     */
    void _debug_setAnimation(bool enabled, int duration_ms = 200, int steps = 12);

private:
    struct Node
    {
        std::string name;
        cv::Size desired{640, 480};    // 期望尺寸（仅参考）
        cv::Rect lastRect{0, 0, 0, 0}; // 上一次实际位置（用于动画）
    };

    std::vector<Node> nodes_;
    cv::Size screen_{1920, 1080};
    cv::Size minWin_{240, 160};

    // 内部动画控制（外部不可见）
    bool animate_ = false;
    int animDurationMs_ = 200; // 总时长
    int animSteps_ = 12;       // 插值帧数

    // 计算网格并布局；若不可行则进入覆盖级联
    void relayout_();

    // 将窗口从 lastRect 过渡到 target（含可选动画），并记录 lastRect
    void applyRect_(Node &n, const cv::Rect &target);
};
