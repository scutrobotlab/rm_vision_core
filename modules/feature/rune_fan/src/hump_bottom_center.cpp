#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_hump_param.h"
#include "vc/math/geom_utils.hpp"

using namespace std;
using namespace cv;

BottomCenterHump::BottomCenterHump(const Point2f &_center, const Point2f &direction, int _idx)
{
    center = _center;
    this->direction = direction;
    idx = _idx;
}

vector<BottomCenterHump> BottomCenterHump::getBottomCenterHump(const vector<Point> &contour, const Point2f &contour_center, const vector<TopHump> &top_humps)
{
    if (contour.size() < 30)
        return {};
    if (top_humps.size() != 3)
        VC_THROW_ERROR("Size of the \" top_humps \" are not equal to 3. (size = %zu)", top_humps.size());

    vector<BottomCenterHump> humps;
    if (!getAllHumps(contour, top_humps, humps) || !filter(contour, top_humps, humps) || humps.size() != 1)
        return {};

    humps[0].vertex = humps[0].getCenter();
    return humps;
}

bool BottomCenterHump::getAllHumps(const vector<Point> &contour, const vector<TopHump> &top_humps, vector<BottomCenterHump> &humps)
{
    if (contour.size() < 30)
        return false;
    if (top_humps.size() != 3)
        VC_THROW_ERROR("Size of the \" top_humps \" are not equal to 3. (size = %zu)", top_humps.size());

    Point2f line_center = top_humps[1].getCenter();
    Point2f line_direction = -top_humps[1].getDirection();
    vector<BottomCenterHump> found_humps;

    Point last_point = contour.back();
    int idx = 0;
    for (auto &point : contour)
    {
        Point2f v1 = static_cast<Point2f>(point) - line_center;
        Point2f v2 = static_cast<Point2f>(last_point) - line_center;
        if ((v1.x * line_direction.y - v1.y * line_direction.x) * (v2.x * line_direction.y - v2.y * line_direction.x) < 0)
        {
            found_humps.emplace_back((v1 + v2) / 2.0f + line_center, line_direction, idx);
        }
        last_point = point;
        idx++;
    }

    if (found_humps.empty())
        return false;
    humps = move(found_humps);
    return true;
}

bool BottomCenterHump::filter(const vector<Point> &contour, const vector<TopHump> &top_humps, vector<BottomCenterHump> &humps)
{
    if (contour.size() < 30 || humps.empty())
    {
        humps = {};
        return false;
    }
    if (top_humps.size() != 3)
        VC_THROW_ERROR("Size of the \" top_humps \" are not equal to 3. (size = %zu)", top_humps.size());

    for (auto it1 = humps.begin(); it1 != humps.end(); it1++)
    {
        for (auto it2 = it1 + 1; it2 != humps.end();)
        {
            if (getDist(it1->getCenter(), it2->getCenter()) < rune_fan_hump_param.BOTTOM_CENTER_HUMP_MIN_INTERVAL)
                it2 = humps.erase(it2);
            else
                it2++;
        }
    }
    if (humps.empty())
        return false;

    Point2f line_center = top_humps[1].getCenter();
    Point2f line_direction = -top_humps[1].getDirection();
    auto bottom_it = max_element(humps.begin(), humps.end(), [&](const BottomCenterHump &h1, const BottomCenterHump &h2)
                                 { return (h1.getCenter() - line_center).dot(line_direction) < (h2.getCenter() - line_center).dot(line_direction); });

    humps = {*bottom_it};
    float delta_angle = getVectorMinAngle(line_direction, humps[0].getCenter() - line_center, DEG);
    if (delta_angle > rune_fan_hump_param.BOTTOM_CENTER_HUMP_MAX_DELTA_ANGLE)
    {
        humps = {};
        return false;
    }
    return true;
}
