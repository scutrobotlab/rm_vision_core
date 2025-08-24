#include <numeric>
#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_hump_param.h"
#include "vc/math/geom_utils.hpp"

using namespace std;
using namespace cv;

SideHump::SideHump(const Point2f &_direction, const Point2f &_center)
{
    direction = _direction;
    center = _center;
}

// 获取侧边突起点
vector<SideHump> SideHump::getSideHumps(
    const vector<Point> &contour_plus,
    const Point2f &contour_center,
    const vector<TopHump> &top_humps,
    const vector<BottomCenterHump> &bottom_center_humps,
    vector<tuple<Line, Line>> &line_pairs)
{
    if (line_pairs.size() < 2)
        return {};

    vector<SideHump> humps;
    if (!getAllHumps(top_humps, line_pairs, humps) || humps.size() < 2)
        return {};

    for (auto &hump : humps)
        setVertex(hump, contour_plus);
    filter(humps);
    if (humps.size() < 2)
        return {};

    vector<tuple<vector<SideHump>, double>> hump_groups;

    // 构造突起点组合
    for (size_t i = 0; i < humps.size() - 1; ++i)
    {
        for (size_t j = i + 1; j < humps.size(); ++j)
        {
            vector<SideHump> temp_humps = {humps[i], humps[j]};
            double delta = 0;
            if (make_SideHumps(temp_humps, top_humps, contour_center, delta))
                hump_groups.emplace_back(temp_humps, delta);
        }
    }

    if (hump_groups.empty())
        return {};

    return std::get<0>(*min_element(hump_groups.begin(), hump_groups.end(),
                       [](const auto &a, const auto &b)
                       { return get<1>(a) < get<1>(b); }));
}

// 获取所有候选侧边突起点
bool SideHump::getAllHumps(const vector<TopHump> &top_humps,
                           const vector<tuple<Line, Line>> &line_pairs,
                           vector<SideHump> &humps)
{
    humps.clear();
    if (top_humps.size() != 3)
    {
        VC_THROW_ERROR("top_humps size != 3 (size=%zu)", top_humps.size());
        return false;
    }

    unordered_set<int> line_pair_idx_set;
    for (int i = 0; i < line_pairs.size(); ++i)
        line_pair_idx_set.insert(i);
    for (auto &top_hump : top_humps)
        line_pair_idx_set.erase(top_hump.getLinePairIdx());

    auto isSideHump = [&](const Point2f &top_center, const Point2f &top_dir, const Point2f &center, const Point2f &dir)
    {
        if (getVectorMinAngle(top_dir, dir, DEG) > rune_fan_hump_param.SIDE_HUMP_MAX_ANGLE_DELTA)
            return false;
        Point2f v1 = top_dir, v2 = top_center - center, v3 = dir;
        if (v1.cross(v2) * v2.cross(v3) < 0)
            return false;
        if (getVectorMinAngle(v1, v2, DEG) > rune_fan_hump_param.SIDE_HUMP_MAX_ANGLE_DELTA / 2.0 ||
            getVectorMinAngle(v2, v3, DEG) > rune_fan_hump_param.SIDE_HUMP_MAX_ANGLE_DELTA / 2.0)
            return false;
        return true;
    };

    Point2f l_center = top_humps[0].getCenter(), l_dir = top_humps[0].getDirection();
    Point2f r_center = top_humps[2].getCenter(), r_dir = top_humps[2].getDirection();

    for (auto &idx : line_pair_idx_set)
    {
        auto &[up_line, down_line] = line_pairs[idx];
        Point2f center = (up_line.center + down_line.center) / 2.0;
        Point2f dir(cos(deg2rad(up_line.angle)), sin(deg2rad(up_line.angle)));

        float to_l = getDist(center, l_center);
        float to_r = getDist(center, r_center);

        if (to_l < to_r && isSideHump(l_center, l_dir, center, dir))
        {
            humps.emplace_back(dir, getLineIntersection(l_center, l_center + l_dir, center, center + dir));
        }
        else if (to_r <= to_l && isSideHump(r_center, r_dir, center, dir))
        {
            humps.emplace_back(dir, getLineIntersection(r_center, r_center + r_dir, center, center + dir));
        }
    }

    return humps.size() >= 2;
}

// 设置突起点的顶点位置
bool SideHump::setVertex(SideHump &hump, const vector<Point> &contour_plus)
{
    hump.vertex = hump.center;
    return true;
}

// 过滤几乎重合的突起点
bool SideHump::filter(vector<SideHump> &humps)
{
    for (auto it1 = humps.begin(); it1 != humps.end(); ++it1)
    {
        for (auto it2 = it1 + 1; it2 != humps.end();)
        {
            if (getDist(it1->getCenter(), it2->getCenter()) < 10)
                it2 = humps.erase(it2);
            else
                ++it2;
        }
    }
    return true;
}

// 确定左右侧突起点
bool SideHump::make_SideHumps(vector<SideHump> &humps, const vector<TopHump> &top_humps, const Point2f &contour_center, double &delta)
{
    if (humps.size() != 2)
    {
        VC_THROW_ERROR("humps size != 2 (size=%zu)", humps.size());
        return false;
    }

    Point2f fan_dir = top_humps[1].getDirection();
    Point2f fan_center = top_humps[1].getCenter();

    double cross0 = (humps[0].getCenter() - fan_center).cross(fan_dir);
    double cross1 = (humps[1].getCenter() - fan_center).cross(fan_dir);

    if (cross0 * cross1 > 0)
        return false;

    if (cross0 < 0)
        swap(humps[0], humps[1]);
    delta = 0;
    return true;
}
