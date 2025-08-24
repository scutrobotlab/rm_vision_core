#include "vc/core/debug_tools/window_auto_layout.h"

WindowAutoLayout::Ptr WindowAutoLayout::get()
{
    static WindowAutoLayout::Ptr instance = []() -> Ptr
    {
        auto result = std::make_shared<WindowAutoLayout>(cv::Size(1920, 1080), cv::Size(320, 240));
        result->_debug_setAnimation(false, 30, 15);
        return result;
    }();
    return instance;
}

WindowAutoLayout::WindowAutoLayout(cv::Size screen, cv::Size minWin)
    : screen_(screen), minWin_(minWin) {}

void WindowAutoLayout::setScreenSize(const cv::Size &s)
{
    screen_ = s;
    relayout_();
}

/** @brief 设置窗口最小阈值（小于该尺寸则转为覆盖模式） */
void WindowAutoLayout::setMinWindowSize(const cv::Size &s)
{
    minWin_ = s;
    relayout_();
}

void WindowAutoLayout::addWindow(const std::string &name, cv::Size desired, int flags)
{
    if (hasWindow(name))
        return; // 已存在则忽略
    Node n;
    n.name = name;
    n.desired = desired;
    n.lastRect = cv::Rect(0, 0, desired.width, desired.height);
    nodes_.push_back(n);
    cv::namedWindow(name, flags);
    // 初始放置在左上角，随后由 relayout_() 统一排布
    cv::resizeWindow(name, desired.width, desired.height);
    cv::moveWindow(name, 0, 0);
    relayout_();
}

/** @brief 移除并销毁窗口 */
void WindowAutoLayout::removeWindow(const std::string &name)
{
    auto it = std::find_if(nodes_.begin(), nodes_.end(), [&](const Node &n)
                           { return n.name == name; });
    if (it != nodes_.end())
    {
        cv::destroyWindow(it->name);
        nodes_.erase(it);
        relayout_();
    }
}

/** @brief 查询窗口是否存在 */
bool WindowAutoLayout::hasWindow(const std::string &name) const
{
    return std::any_of(nodes_.begin(), nodes_.end(), [&](const Node &n)
                       { return n.name == name; });
}

/** @brief 强制重新布局（例如外部改变了屏幕尺寸） */
void WindowAutoLayout::relayout() 
{
    relayout_(); 
}

/**
 * @brief 设置内部动画（对外 API 不暴露业务参数，仅用于开发/调试）
 * @param enabled 是否启用
 * @param duration_ms 单次过渡时长（毫秒）
 * @param steps 插值帧数（越大越顺滑）
 */
void WindowAutoLayout::_debug_setAnimation(bool enabled, int duration_ms, int steps)
{
    animate_ = enabled;
    animDurationMs_ = std::max(0, duration_ms);
    animSteps_ = std::max(1, steps);
}

// 计算网格并布局；若不可行则进入覆盖级联
void WindowAutoLayout::relayout_()
{
    if (nodes_.empty())
        return;

    // 尝试 1..N 列的网格，找到第一个满足 minWin 的方案
    const int N = static_cast<int>(nodes_.size());
    bool placed = false;
    for (int cols = 1; cols <= N; ++cols)
    {
        int rows = static_cast<int>(std::ceil(N / static_cast<double>(cols)));
        int cellW = screen_.width / cols;
        int cellH = screen_.height / rows;
        if (cellW < minWin_.width || cellH < minWin_.height)
            continue; // 不满足最小阈值

        // 满足阈值 —— 采用网格布局
        for (int i = 0; i < N; ++i)
        {
            int r = i / cols;
            int c = i % cols;
            int w = std::min(cellW, nodes_[i].desired.width);
            int h = std::min(cellH, nodes_[i].desired.height);
            // 在单元格内左上对齐，可改为居中：x+ (cellW-w)/2, y+(cellH-h)/2
            int x = c * cellW;
            int y = r * cellH;
            cv::Rect target(x, y, w, h);
            applyRect_(nodes_[i], target);
        }
        placed = true;
        break;
    }

    if (!placed)
    {
        // 覆盖模式：级联（cascade），每个窗口偏移固定步长
        const int step = std::min(std::max(minWin_.width / 8, 24), 80);
        for (int i = 0; i < N; ++i)
        {
            int x = (i * step) % std::max(1, screen_.width - minWin_.width);
            int y = (i * step) % std::max(1, screen_.height - minWin_.height);
            cv::Rect target(x, y, minWin_.width, minWin_.height);
            applyRect_(nodes_[i], target);
        }
    }
}

// 将窗口从 lastRect 过渡到 target（含可选动画），并记录 lastRect
void WindowAutoLayout::applyRect_(Node &n, const cv::Rect &target)
{
    cv::Rect from = n.lastRect;
    if (from.width <= 0 || from.height <= 0)
    {
        // 首次布局：使用目标作为起点，避免大幅动画跳跃
        from = target;
    }

    if (animate_)
    {
        // 线性插值动画
        for (int s = 1; s <= animSteps_; ++s)
        {
            double t = static_cast<double>(s) / animSteps_;
            int x = static_cast<int>(std::lround(from.x + t * (target.x - from.x)));
            int y = static_cast<int>(std::lround(from.y + t * (target.y - from.y)));
            int w = static_cast<int>(std::lround(from.width + t * (target.width - from.width)));
            int h = static_cast<int>(std::lround(from.height + t * (target.height - from.height)));
            cv::resizeWindow(n.name, std::max(1, w), std::max(1, h));
            cv::moveWindow(n.name, x, y);
            // 粗略的节流：依赖 waitKey 让主循环泵事件（避免卡死）
            cv::waitKey(std::max(1, animDurationMs_ / animSteps_));
        }
    }
    else
    {
        cv::resizeWindow(n.name, std::max(1, target.width), std::max(1, target.height));
        cv::moveWindow(n.name, target.x, target.y);
    }

    n.lastRect = target;
}
