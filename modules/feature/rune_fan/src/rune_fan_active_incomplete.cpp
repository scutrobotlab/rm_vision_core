#include <numeric>
#include "vc/feature/rune_fan_active.h"
#include "vc/feature/rune_fan_param.h"
#include "vc/feature/rune_fan_hump_param.h"

using namespace std;
using namespace cv;

inline bool getHumps(const vector<Point> &contour, vector<TopHump> &top_humps)
{
    if (contour.size() < 30)
        return false;
    vector<Point> contour_plus(contour.begin(), contour.end());
    contour_plus.insert(contour_plus.end(), contour.begin(), contour.begin() + contour.size() / 3);
    Mat angles_mat = RuneFanActive::getAngles(contour_plus);
    Mat gradient_mat = RuneFanActive::getGradient(angles_mat);
    vector<tuple<Line, Line>> line_pairs;
    RuneFanActive::getLinePairs(contour_plus, angles_mat, gradient_mat, line_pairs);

    vector<TopHump> humps;
    for (int i = 0; i < line_pairs.size(); i++)
    {
        auto &[up_line, down_line] = line_pairs[i];
        Point2f direction(cos(deg2rad(up_line.angle)), sin(deg2rad(up_line.angle)));
        Point2f center = (contour_plus[up_line.start_idx] + contour_plus[down_line.start_idx]) / 2.0;
        humps.emplace_back(up_line.start_idx, up_line.end_idx, down_line.start_idx, down_line.end_idx, i, direction, center);
    }
    for (auto &h : humps)
        TopHump::setVertex(h, contour_plus);
    TopHump::filter(contour_plus, humps);
    top_humps = humps;
    return true;
}

inline bool isOverlap(const RuneFanActive_ptr &fan1, const RuneFanActive_ptr &fan2)
{
    return (fan1->getRotatedRect().boundingRect() & fan2->getRotatedRect().boundingRect()).area() > 0;
}

inline bool filterFan(const vector<FeatureNode_ptr> &fans, vector<FeatureNode_ptr> &filtered_fans, const Point2f &rotate_center)
{
    unordered_set<FeatureNode_ptr> used_fans(fans.begin(), fans.end());
    for (auto &fan : fans)
    {
        if (used_fans.find(fan) == used_fans.end())
            continue;
        auto rune_fan = RuneFanActive::cast(fan);
        double angle = getVectorMinAngle(rune_fan->getImageCache().getDirection(), rotate_center - rune_fan->getImageCache().getCenter(), DEG);
        if (abs(angle) > rune_fan_param.ACTIVE_MAX_DIRECTION_DELTA_INCOMPLETE)
            used_fans.erase(fan);
    }
    if (used_fans.empty())
        return false;
    filtered_fans.insert(filtered_fans.end(), used_fans.begin(), used_fans.end());
    return true;
}

bool RuneFanActive::find_incomplete(vector<FeatureNode_ptr> &fans, const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, const unordered_set<size_t> &mask, const Point2f &rotate_center, unordered_map<FeatureNode_cptr, unordered_set<size_t>> &used_contour_idxs)
{
    unordered_set<size_t> pending_idxs;
    for (size_t i = 0; i < contours.size(); i++)
        if (mask.find(i) == mask.end() && contours[i]->points().size() >= 6 && hierarchy[i][3] == -1)
            pending_idxs.insert(i);

    for (auto it = pending_idxs.begin(); it != pending_idxs.end();)
    {
        if (contours[*it]->area() < rune_fan_param.ACTIVE_MIN_AREA_INCOMPLETE)
            it = pending_idxs.erase(it);
        else
            ++it;
    }
    if (pending_idxs.empty())
        return false;

    for (auto it = pending_idxs.begin(); it != pending_idxs.end();)
    {
        double ratio = contours[*it]->area() / pow(contours[*it]->perimeter(), 2);
        if (ratio > rune_fan_param.ACTIVE_MAX_AREA_PERIMETER_RATIO_INCOMPLETE)
            it = pending_idxs.erase(it);
        else
            ++it;
    }

    static auto getContourIdx = [](const Contour_cptr &c, const vector<Contour_cptr> &contours)->int
    {
        auto it = std::find(contours.begin(), contours.end(), c);
        if (it != contours.end())
            return std::distance(contours.begin(), it);
        VC_THROW_ERROR("Contour not found");
        return -1;
    };

    vector<tuple<TopHump, Contour_cptr>> all_humps;
    for (auto idx : pending_idxs)
    {
        vector<TopHump> temp_humps;
        getHumps(contours[idx]->points(), temp_humps);
        for (auto &h : temp_humps)
            all_humps.push_back({h, contours[idx]});
    }
    if (all_humps.size() < 3)
        return false;

    vector<Point2f> hump_centers(all_humps.size());
    for (size_t i = 0; i < all_humps.size(); i++)
        hump_centers[i] = get<0>(all_humps[i]).getCenter();

    Mat hump_centers_mat(hump_centers.size(), 2, CV_32F);
    for (size_t i = 0; i < hump_centers.size(); i++)
    {
        hump_centers_mat.at<float>(i, 0) = hump_centers[i].x;
        hump_centers_mat.at<float>(i, 1) = hump_centers[i].y;
    }

    flann::Index kd_tree(hump_centers_mat, flann::KDTreeIndexParams(1));

    unordered_map<RuneFanActive_ptr, unordered_set<size_t>> used_contour_idxs_temp;
    for (size_t i = 0; i < all_humps.size(); i++)
    {
        Mat query = (Mat_<float>(1, 2) << hump_centers[i].x, hump_centers[i].y);
        int max_results = 50;
        Mat indices(1, max_results, CV_32S), dists(1, max_results, CV_32F);
        int found = kd_tree.radiusSearch(query, indices, dists, 500 * 500, max_results, flann::SearchParams(32));

        for (size_t n = 0; n < found; ++n)
        {
            size_t j = indices.at<int>(0, n);
            if (j <= i || sqrt(dists.at<float>(0, n)) > 300)
                continue;
            auto &h1 = get<0>(all_humps[i]), &h2 = get<0>(all_humps[j]);
            if (getVectorMinAngle(h1.getDirection(), h2.getDirection(), DEG) > 30)
                continue;

            for (size_t n2 = 0; n2 < found; ++n2)
            {
                size_t k = indices.at<int>(0, n2);
                if (k <= i || k == j || sqrt(dists.at<float>(0, n2)) > 300)
                    continue;
                auto &h3 = get<0>(all_humps[k]);
                if (getVectorMinAngle(h1.getDirection(), h3.getDirection(), DEG) > 30)
                    continue;
                if (getVectorMinAngle(h2.getDirection(), h3.getDirection(), DEG) > 30)
                    continue;

                auto p_fan = make_feature(all_humps[i], all_humps[j], all_humps[k]);
                if (p_fan)
                {
                    fans.push_back(p_fan);
                    used_contour_idxs_temp[p_fan].insert(getContourIdx(get<1>(all_humps[i]), contours));
                    used_contour_idxs_temp[p_fan].insert(getContourIdx(get<1>(all_humps[j]), contours));
                    used_contour_idxs_temp[p_fan].insert(getContourIdx(get<1>(all_humps[k]), contours));
                }
            }
        }
    }

    vector<FeatureNode_ptr> filtered_fans;
    filterFan(fans, filtered_fans, rotate_center);
    return true;
}

RuneFanActive::RuneFanActive(const vector<Contour_cptr> &contours, const vector<Point2f> &top_hump_corners, const Point2f &direction)
{
    vector<Point> contour_temp;
    for (auto &c : contours)
        contour_temp.insert(contour_temp.end(), c->begin(), c->end());
    vector<Point> hull_contour_temp;
    convexHull(contour_temp, hull_contour_temp);
    auto contour = ContourWrapper<int>::make_contour(hull_contour_temp);
    RotatedRect fit_ellipse = contour->fittedEllipse();
    float width = max(fit_ellipse.size.width, fit_ellipse.size.height);
    float height = min(fit_ellipse.size.width, fit_ellipse.size.height);
    Point2f center = fit_ellipse.center;

    vector<Point2f> corners(top_hump_corners.begin(), top_hump_corners.end());
    Point2f top_center = top_hump_corners[1];
    float max_projection = 0;
    for (auto &point : contour->points())
    {
        float projection = getProjection(static_cast<Point2f>(point) - top_center, direction);
        max_projection = max(max_projection, projection);
    }
    corners.emplace_back(top_center + max_projection * direction);

    setActiveFlag(true);
    setTopHumpCorners(top_hump_corners);
    setRotatedRect(fit_ellipse);

    auto &image_info = getImageCache();
    image_info.setContours(vector<Contour_cptr>{contour});
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCenter(center);
    image_info.setCorners(corners);
    image_info.setDirection(direction);
}

inline Point2f getHumpCenter(const array<tuple<TopHump, Contour_cptr>, 3> &humps)
{
    unordered_set<Contour_cptr> contours;
    for (auto &[h, c] : humps)
        contours.insert(c);
    if (contours.size() == 3)
    {
        Point2f ave_point = (get<0>(humps[0]).getVertex() + get<0>(humps[1]).getVertex() + get<0>(humps[2]).getVertex()) / 3.0;
        array<float, 3> distances;
        for (int i = 0; i < 3; i++)
            distances[i] = getDist(ave_point, get<0>(humps[i]).getVertex());
        return get<1>(humps[distance(distances.begin(), min_element(distances.begin(), distances.end()))])->center();
    }
    else if (contours.size() == 2)
    {
        auto hull_contour = ContourWrapper<int>::getConvexHull({contours.begin(), contours.end()});
        return hull_contour->fittedEllipse().center;
    }
    else if (contours.size() == 1)
        return (*contours.begin())->fittedEllipse().center;
    VC_THROW_ERROR("humps size is not equal to 3");
    return Point2f(0, 0);
}

shared_ptr<RuneFanActive> RuneFanActive::make_feature(const tuple<TopHump, Contour_cptr> &h1, const tuple<TopHump, Contour_cptr> &h2, const tuple<TopHump, Contour_cptr> &h3)
{
    auto &[hump_1_obj, contour_1] = h1;
    auto &[hump_2_obj, contour_2] = h2;
    auto &[hump_3_obj, contour_3] = h3;

    Point2f contours_center = getHumpCenter({h1, h2, h3});
    TopHumpCombo hump_combo(hump_1_obj, hump_2_obj, hump_3_obj);

    double max_distance = max({getDist(hump_1_obj.getVertex(), hump_2_obj.getVertex()),
                               getDist(hump_1_obj.getVertex(), hump_3_obj.getVertex()),
                               getDist(hump_2_obj.getVertex(), hump_3_obj.getVertex())});
    if (max_distance == 0)
        return nullptr;

    double max_side_length = max({max(contour_1->fittedEllipse().size.width, contour_1->fittedEllipse().size.height),
                                  max(contour_2->fittedEllipse().size.width, contour_2->fittedEllipse().size.height),
                                  max(contour_3->fittedEllipse().size.width, contour_3->fittedEllipse().size.height)});
    if (max_side_length == 0 || max_distance > 3.0 * max_side_length)
        return nullptr;

    double angle_delta = 1e5;
    if (!TopHump::make_TopHumps(hump_combo, contours_center, angle_delta))
        return nullptr;

    vector<Contour_cptr> fan_contours{contour_1, contour_2, contour_3};
    auto &top_humps = hump_combo.humps();
    Point2f direction = -1 * top_humps[1].getDirection();
    vector<Point2f> top_humps_corners;
    for (auto &h : top_humps)
        top_humps_corners.push_back(h.getVertex());

    auto fan = make_shared<RuneFanActive>(fan_contours, top_humps_corners, direction);
    if (fan && !fan->isSetError())
        fan->setError(angle_delta);
    return fan;
}
