#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_hump_param.h"
#include "vc/core/debug_tools.h"
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define RUNE_FAN_DEBUG 1

struct TopHumpDebugParam
{
    bool ENABLE_DEBUG = false;
    YML_INIT(TopHumpDebugParam, YML_ADD_PARAM(ENABLE_DEBUG);)
};
inline TopHumpDebugParam top_hump_debug_param;

TopHump::TopHump(int _up_start_idx, int _up_end_idx, int _down_start_idx, int _down_end_idx, int _line_pair_idx, const cv::Point2f &_direction, const cv::Point2f &_center)
    : up_start_idx(_up_start_idx), up_end_idx(_up_end_idx), down_start_idx(_down_start_idx), down_end_idx(_down_end_idx),
      line_pair_idx(_line_pair_idx), up_itertaion_num(_up_end_idx - _up_start_idx),
      down_itertaion_num(_down_end_idx - _down_start_idx), end_iteration_up_idx(_up_end_idx), current_state(DOWN)
{
    direction = _direction;
    center = _center;
}

TopHump::TopHump(int up_idx, int down_idx, state find_state, const cv::Point2f &_direction)
    : current_state(find_state), up_itertaion_num(0), down_itertaion_num(0), end_iteration_up_idx(up_idx)
{
    direction = _direction;
    if (find_state == UP)
    {
        up_start_idx = up_idx;
        down_start_idx = down_idx;
        up_end_idx = down_end_idx = 0;
    }
    else if (find_state == DOWN)
    {
        up_end_idx = up_idx;
        down_end_idx = down_idx;
        up_start_idx = down_start_idx = 0;
    }
}

vector<TopHump> TopHump::getTopHumps(const vector<Point> &contour_plus, const Point2f &contour_center, const vector<tuple<Line, Line>> &line_pairs)
{
    if (contour_plus.size() < 20)
    {
        VC_WARNING_INFO("fan active get_top_humps : 轮廓长度小于20,无法进行查找,轮廓数量：%zu", contour_plus.size());
        return {};
    }
    vector<TopHump> humps{};
    TopHump::getAllHumps2(contour_plus, line_pairs, humps);
    if (humps.size() < 3)
    {
        VC_WARNING_INFO("fan active get_top_humps : 收集到的突起点数不足3个,无法进行查找,轮廓数量：%zu", contour_plus.size());
        return {};
    }
    TopHump::filter(contour_plus, humps);
    if (humps.size() < 3)
    {
        VC_WARNING_INFO("fan active get_top_humps : 过滤后的突起点数不足3个,无法进行查找,轮廓数量：%zu", contour_plus.size());
        return {};
    }
    vector<tuple<TopHumpCombo, double>> hump_combos{};
    for (size_t i = 0; i < humps.size() - 2; ++i)
        for (size_t j = i + 1; j < humps.size() - 1; ++j)
            for (size_t k = j + 1; k < humps.size(); ++k)
            {
                TopHumpCombo hump_combo = {humps[i], humps[j], humps[k]};
                double delta = 0;
                if (TopHump::make_TopHumps(hump_combo, contour_center, delta))
                    hump_combos.emplace_back(hump_combo, delta);
            }
    if (hump_combos.empty())
        return {};
    auto [best_combo, delta] = *min_element(hump_combos.begin(), hump_combos.end(), [](const auto &a, const auto &b)
                                            { return get<1>(a) < get<1>(b); });
    return best_combo.getHumpsVector();
}

float gaussian(float x, float sigma) { return (1.0 / (sqrt(2 * M_PI) * sigma)) * exp(-x * x / (2 * sigma * sigma)); }
void normalize(vector<float> &kernel)
{
    float sum = 0;
    for (float k : kernel)
        sum += k;
    for (float &k : kernel)
        k /= sum;
}
vector<float> createGaussianKernel(int size, float sigma)
{
    vector<float> kernel(size);
    int center = size / 2;
    for (int i = 0; i < size; ++i)
        kernel[i] = gaussian(i - center, sigma);
    normalize(kernel);
    return kernel;
}

void TopHump::update(const TopHump &hump)
{
    current_state = hump.current_state;
    if (current_state == UP)
    {
        up_start_idx = min(up_start_idx, hump.up_start_idx);
        down_start_idx = min(down_start_idx, hump.down_start_idx);
        up_itertaion_num++;
    }
    else if (current_state == DOWN)
    {
        up_end_idx = max(up_end_idx, hump.up_end_idx);
        down_end_idx = max(down_end_idx, hump.down_end_idx);
        down_itertaion_num++;
    }
    end_iteration_up_idx = hump.end_iteration_up_idx;
    int iteration_num = up_itertaion_num + down_itertaion_num;
    float ratio_old = float(iteration_num - 1) / iteration_num, ratio_new = 1.0f / iteration_num;
    direction = direction * ratio_old + hump.direction * ratio_new;
}

inline bool TopHump::getAllHumps2(const vector<Point> &contours_plus, const vector<tuple<Line, Line>> &line_pairs, vector<TopHump> &humps)
{
    humps.clear();
    for (int i = 0; i < line_pairs.size(); i++)
    {
        auto &[up_line, down_line] = line_pairs[i];
        int up_start_idx = up_line.start_idx, up_end_idx = up_line.end_idx, down_start_idx = down_line.start_idx, down_end_idx = down_line.end_idx;
        Point2f direction = Point2f(cos(deg2rad(up_line.angle)), sin(deg2rad(up_line.angle)));
        Point2f center = (contours_plus[up_start_idx] + contours_plus[down_start_idx]) / 2.0;
        humps.emplace_back(up_start_idx, up_end_idx, down_start_idx, down_end_idx, i, direction, center);
    }
    for (auto &hump : humps)
        setVertex(hump, contours_plus);
    return humps.size() >= 3;
}

bool TopHump::filter(const vector<Point> &contour_plus, vector<TopHump> &humps)
{
    if (humps.empty())
        return false;
    for (auto it_1 = humps.begin(); it_1 != humps.end(); it_1++)
        for (auto it_2 = it_1 + 1; it_2 != humps.end();)
        {
            if (getVectorMinAngle(it_1->getDirection(), it_2->getDirection(), DEG) < rune_fan_hump_param.TOP_HUMP_MAX_ALIGNMENT_DELTA)
            {
                if (getDist(it_1->getVertex(), it_2->getVertex()) < rune_fan_hump_param.TOP_HUMP_MIN_INTERVAL)
                    it_2 = humps.erase(it_2);
                else
                    it_2++;
            }
            else
            {
                if (getDist(it_1->getVertex(), it_2->getVertex()) < 2 * rune_fan_hump_param.TOP_HUMP_MIN_INTERVAL)
                    it_2 = humps.erase(it_2);
                else
                    it_2++;
            }
        }
    return true;
}

bool TopHump::make_TopHumps(TopHumpCombo &hump_combo, const Point2f &contour_center, double &delta)
{
    auto &humps = hump_combo.humps();
    vector<TopHump> humps_to_classify(humps.begin(), humps.end());
    TopHump left_h, right_h, center_h;
    Point2f aveVertex{};
    for (auto &h : humps)
        aveVertex += h.getVertex();
    aveVertex /= static_cast<float>(humps.size());
    auto center_hump_it = min_element(humps_to_classify.begin(), humps_to_classify.end(), [&](const TopHump &h1, const TopHump &h2)
                                      { return getDist(h1.getVertex(), aveVertex) < getDist(h2.getVertex(), aveVertex); });
    center_h = *center_hump_it;
    humps_to_classify.erase(center_hump_it);
    if (humps_to_classify.size() != 2)
        return false;
    Point2f c0 = humps_to_classify[0].getVertex() - contour_center, c1 = humps_to_classify[1].getVertex() - contour_center;
    if (c0.x * c1.y - c0.y * c1.x > 0)
    {
        left_h = humps_to_classify[0];
        right_h = humps_to_classify[1];
    }
    else
    {
        left_h = humps_to_classify[1];
        right_h = humps_to_classify[0];
    }

    if (HumpDetector::CheckCollinearity(left_h.getVertex(), center_h.getVertex(), right_h.getVertex()) > rune_fan_hump_param.TOP_HUMP_MAX_COLLINEAR_DELTA)
        return false;

    if (getVectorMinAngle(center_h.getDirection(), center_h.getVertex() - contour_center, DEG) > rune_fan_hump_param.TOP_HUMP_MAX_DIRECTION_DELTA)
        return false;

    if (HumpDetector::CheckAlignment(left_h.getDirection(), center_h.getDirection(), right_h.getDirection()) > rune_fan_hump_param.TOP_HUMP_MAX_ALIGNMENT_DELTA)
        return false;

    double distance_ratio = getDist(left_h.getVertex(), center_h.getVertex()) / getDist(center_h.getVertex(), right_h.getVertex());
    if (distance_ratio < 1.0)
        distance_ratio = 1.0 / distance_ratio;
    if (distance_ratio > rune_fan_hump_param.TOP_HUMP_MAX_DISTANCE_RATIO)
        return false;

    Vec4f line;
    fitLine(vector<Point2f>{left_h.getVertex(), center_h.getVertex(), right_h.getVertex()}, line, DIST_L2, 0, 0.01, 0.01);
    Point2f ave_direction = (left_h.getDirection() + center_h.getDirection() + right_h.getDirection()) / 3.0;
    double line_direction_delta = getVectorMinAngle(Point2f(line[0], line[1]), ave_direction, DEG);
    line_direction_delta = line_direction_delta > 90 ? 180 - line_direction_delta : line_direction_delta;

    if (line_direction_delta < rune_fan_hump_param.TOP_HUMP_MIN_LINE_DIRECTION_DELTA)
        return false;
        
    delta = pow(HumpDetector::CheckAlignment(left_h.getDirection(), center_h.getDirection(), right_h.getDirection()), 2);
    humps = {left_h, center_h, right_h};
    return true;
}

bool TopHump::setVertex(TopHump &hump, const vector<Point> &contour_plus)
{
    if (contour_plus.size() < 20)
        return false;
    auto farthest_point = max_element(contour_plus.begin() + hump.up_start_idx, contour_plus.begin() + hump.down_end_idx, [&](const Point &p1, const Point &p2)
                                      { return getProjection(Point2f(p1) - hump.getCenter(), hump.getDirection()) < getProjection(Point2f(p2) - hump.getCenter(), hump.getDirection()); });
    hump.vertex = hump.getCenter() + getProjectionVector(Point2f(*farthest_point) - hump.getCenter(), hump.getDirection());
    return true;
}
