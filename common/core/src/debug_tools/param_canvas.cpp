#include <algorithm>
#include <limits>
#include "vc/core/debug_tools/param_canvas.h"
#include "vc/core/debug_tools/window_auto_layout.h"

ParamCanvas::ParamCanvas(const std::string &name, int width, int height, int max_points)
    : winname_(name), img_col_(width), img_row_(height),
      margin_left_(60), margin_right_(40), margin_top_(40), margin_bottom_(60),
      num_x_ticks_(10), num_y_ticks_(10), max_points_per_param_(max_points),
      window_init_flag_(false), smooth_alpha_(0.1f),
      smooth_min_y_(0.0f), smooth_max_y_(1.0f),
      img_(height, width, CV_8UC3, cv::Scalar(0, 0, 0)),
      colors_{
          cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0),
          cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255),
          cv::Scalar(100, 200, 50), cv::Scalar(200, 100, 50), cv::Scalar(50, 100, 200),
          cv::Scalar(100, 50, 200)}
{
}

void ParamCanvas::setCanvasSize(int width, int height)
{
    img_col_ = width;
    img_row_ = height;
    img_.create(img_row_, img_col_, CV_8UC3);
}

void ParamCanvas::setTicks(int x_ticks, int y_ticks)
{
    num_x_ticks_ = x_ticks;
    num_y_ticks_ = y_ticks;
}

void ParamCanvas::setMaxPoints(int max_points)
{
    max_points_per_param_ = max_points;
}

void ParamCanvas::setSmoothing(float alpha)
{
    smooth_alpha_ = std::clamp(alpha, 0.01f, 1.0f);
}

void ParamCanvas::addParam(const std::string &key, float value)
{
    auto &dq = param_map_[key];
    if (dq.size() >= max_points_per_param_)
        dq.pop_front();
    dq.push_back(value);
}

void ParamCanvas::addParam(const std::string &key, const std::vector<float> &values, bool overwrite)
{
    auto &dq = param_map_[key];
    if (overwrite)
    {
        dq.clear();
    }
    //计算需要新加入的元素数量
    size_t new_add_num = std::min(values.size(), static_cast<size_t>(max_points_per_param_));
    // 计算需要删除的元素数量
    size_t remove_num = dq.size() + new_add_num - max_points_per_param_;
    if (remove_num > 0)
    {
        dq.erase(dq.begin(), dq.begin() + std::min(remove_num, dq.size()));
    }
    // 若新加入的数据超过最大点数，则只保留最新的 max_points_per_param_ 个点数
    dq.insert(dq.end(), values.begin() + std::max(0, static_cast<int>(values.size()) - max_points_per_param_), values.end());
    // 确保 deque 的大小不超过 max_points_per_param_
    if (dq.size() > max_points_per_param_)
    {
        dq.erase(dq.begin(), dq.begin() + (dq.size() - max_points_per_param_));
    }
}

void ParamCanvas::markX(int x_index, const std::string &label, const cv::Scalar &color)
{
    x_index = std::clamp(x_index, 0, max_points_per_param_ - 1);
    x_marks_.emplace_back(x_index, label, color);
}

void ParamCanvas::markY(float y_value, const std::string &label, const cv::Scalar &color)
{
    y_marks_.emplace_back(y_value, label, color);
}

void ParamCanvas::reset()
{
    param_map_.clear();
}

void ParamCanvas::windowInit()
{
    if (!window_init_flag_)
    {
        // cv::namedWindow(winname_, cv::WINDOW_AUTOSIZE);
        WindowAutoLayout::get()->addWindow(winname_);
        window_init_flag_ = true;

        // 调整窗口显示大小
        if (window_display_width_ > 0 && window_display_height_ > 0)
        {
            cv::resizeWindow(winname_, window_display_width_, window_display_height_);
        }

        // 移动窗口位置
        if (window_pos_x_ >= 0 && window_pos_y_ >= 0)
        {
            cv::moveWindow(winname_, window_pos_x_, window_pos_y_);
        }
    }
}

void ParamCanvas::alignParams()
{
    // 确保所有参数的点数一致,对点数不足的参数，用其最后一次更新的值做填充
    size_t max_size = 0;
    for (const auto &[_, dq] : param_map_)
    {
        max_size = std::max(max_size, dq.size());
    }
    if (max_size == 0)
        return;

    for (auto &[key, dq] : param_map_)
    {
        while (dq.size() < max_size)
            addParam(key, dq.back());
            // addParam(key,-100);
    }
}

void ParamCanvas::clearMarks()
{
    x_marks_.clear();
    y_marks_.clear();
}

void ParamCanvas::updateDisplayRange()
{
    float cur_min = std::numeric_limits<float>::max();
    float cur_max = std::numeric_limits<float>::lowest();

    for (const auto &[_, dq] : param_map_)
    {
        if (!dq.empty())
        {
            auto [min_it, max_it] = std::minmax_element(dq.begin(), dq.end());
            cur_min = std::min(cur_min, *min_it);
            cur_max = std::max(cur_max, *max_it);
        }
    }

    if (cur_min == cur_max)
    {
        cur_min -= 1.f;
        cur_max += 1.f;
    }

    // 指数平滑更新
    smooth_min_y_ = smooth_alpha_ * cur_min + (1.f - smooth_alpha_) * smooth_min_y_;
    smooth_max_y_ = smooth_alpha_ * cur_max + (1.f - smooth_alpha_) * smooth_max_y_;
}

void ParamCanvas::drawAxes(cv::Mat &img, float min_y, float max_y)
{
    cv::line(img, {margin_left_, margin_top_}, {margin_left_, img_row_ - margin_bottom_}, {200, 200, 200});
    cv::line(img, {margin_left_, img_row_ - margin_bottom_}, {img_col_ - margin_right_, img_row_ - margin_bottom_}, {200, 200, 200});

    for (int i = 0; i <= num_y_ticks_; ++i)
    {
        int y = margin_top_ + i * (img_row_ - margin_top_ - margin_bottom_) / num_y_ticks_;
        float val = max_y - i * (max_y - min_y) / num_y_ticks_;
        cv::line(img, {margin_left_ - 5, y}, {margin_left_ + 5, y}, {200, 200, 200});
        cv::putText(img, cv::format("%.2f", val), {5, y + 5}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {200, 200, 200});
    }

    for (int i = 0; i <= num_x_ticks_; ++i)
    {
        int x = margin_left_ + i * (img_col_ - margin_left_ - margin_right_) / num_x_ticks_;
        int x_val = int(i * (max_points_per_param_ - 1) / num_x_ticks_);
        cv::line(img, {x, img_row_ - margin_bottom_ - 5}, {x, img_row_ - margin_bottom_ + 5}, {200, 200, 200});
        cv::putText(img, std::to_string(x_val), {x - 10, img_row_ - margin_bottom_ + 20}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {200, 200, 200});
    }
}

void ParamCanvas::drawParams(cv::Mat &img, float min_y, float max_y)
{
    if (param_map_.empty())
        return;

    auto mapX = [&](int i) {
        return margin_left_ + int(float(i) / (max_points_per_param_ - 1) * (img_col_ - margin_left_ - margin_right_));
    };
    auto mapY = [&](float y) {
        return margin_top_ + int((max_y - y) / (max_y - min_y) * (img_row_ - margin_top_ - margin_bottom_));
    };

    int idx = 0;
    for (const auto &[key, dq] : param_map_)
    {
        if (dq.size() < 2)
            continue;
        cv::Scalar color = colors_[idx++ % colors_.size()];
        for (size_t i = 1; i < dq.size(); ++i)
        {
            cv::line(img, {mapX(i - 1), mapY(dq[i - 1])}, {mapX(i), mapY(dq[i])}, color, 2);
        }
        
        // 在图像的左上角显示参数名称
        cv::putText(img, key, {margin_left_ + 5, margin_top_ + 20 + idx * 15}, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);

        idx++;
    }
}

void ParamCanvas::show()
{
    windowInit();
    img_.setTo(cv::Scalar(30, 30, 30));

    if (param_map_.empty())
    {
        cv::imshow(winname_, img_);
        cv::waitKey(1);
        return;
    }
    alignParams();
    updateDisplayRange();
    drawAxes(img_, smooth_min_y_, smooth_max_y_);
    drawParams(img_, smooth_min_y_, smooth_max_y_);
    do
    {
        // 额外绘制标注线
        for (const auto &[x_idx, label, color] : x_marks_)
        {
            int x = margin_left_ + int(float(x_idx) / (max_points_per_param_ - 1) * (img_col_ - margin_left_ - margin_right_));
            cv::line(img_, {x, margin_top_}, {x, img_row_ - margin_bottom_}, color, 1, cv::LINE_AA);
            if (!label.empty())
            {
                cv::putText(img_, label, {x + 3, margin_top_ + 15}, cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }
        }

        for (const auto &[y_val, label, color] : y_marks_)
        {
            int y = margin_top_ + int((smooth_max_y_ - y_val) / (smooth_max_y_ - smooth_min_y_) * (img_row_ - margin_top_ - margin_bottom_));
            cv::line(img_, {margin_left_, y}, {img_col_ - margin_right_, y}, color, 1, cv::LINE_AA);
            if (!label.empty())
            {
                cv::putText(img_, label, {margin_left_ + 5, y - 3}, cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }
        }
    } while (0);
    
    cv::imshow(winname_, img_);
}
