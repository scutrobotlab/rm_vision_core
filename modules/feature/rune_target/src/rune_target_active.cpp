#include <opencv2/imgproc.hpp>
#include <string>
#include "vc/feature/rune_target_active.h"
#include "vc/feature/rune_target_param.h"
#include "vc/camera/camera_param.h"
using namespace std;
using namespace cv;

RuneTargetActive::RuneTargetActive(const Contour_cptr contour, const vector<Point2f> corners) : RuneTarget(contour, corners) { setActiveFlag(true); }

RuneTargetActive::RuneTargetActive(const Point2f center, const vector<Point2f> corners)
{
    vector<Point2f> temp_contour = corners;
    float width = rune_target_param.ACTIVE_DEFAULT_SIDE, height = rune_target_param.ACTIVE_DEFAULT_SIDE;
    Contour_cptr contour = nullptr;
    if (temp_contour.size() < 3)
    {
        vector<Point> contours_point(temp_contour.begin(), temp_contour.end());
        contour = ContourWrapper<int>::make_contour(contours_point);
    }
    else
    {
        vector<Point2f> hull;
        convexHull(temp_contour, hull);
        vector<Point> contours_point(hull.begin(), hull.end());
        contour = ContourWrapper<int>::make_contour(contours_point);
    }
    setActiveFlag(true);
    auto &image_info = getImageCache();
    image_info.setCenter(center);
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCorners(corners);
    image_info.setContours(vector<Contour_cptr>{contour});
}

inline bool isHierarchyActiveTarget(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t idx)
{
    return hierarchy[idx][3] == -1 && hierarchy[idx][2] != -1;
}

void RuneTargetActive::find(vector<FeatureNode_ptr> &targets, const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, const unordered_set<size_t> &mask, unordered_map<FeatureNode_cptr, unordered_set<size_t>> &used_contour_idxs)
{
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (mask.find(i) != mask.end())
            continue;
        if (isHierarchyActiveTarget(contours, hierarchy, i))
        {
            unordered_set<size_t> temp_used_contour_idxs{};
            auto p_target = RuneTargetActive::make_feature(contours, hierarchy, i, temp_used_contour_idxs);
            if (p_target)
            {
                targets.push_back(p_target);
                used_contour_idxs[p_target] = temp_used_contour_idxs;
            }
        }
    }
}

auto RuneTargetActive::getPnpPoints() const -> tuple<vector<Point2f>, vector<Point3f>, vector<float>>
{
    return make_tuple(vector<Point2f>{getImageCache().getCenter()}, vector<Point3f>{Point3f(0, 0, 0)}, vector<float>{1.0});
}

inline bool checkEllipse(const Contour_cptr &contour)
{
    if (contour->points().size() < 6)
        return false;
    float contour_area = contour->area();
    if (contour_area < rune_target_param.ACTIVE_MIN_AREA || contour_area > rune_target_param.ACTIVE_MAX_AREA)
        return false;
    auto fit_ellipse = contour->fittedEllipse();
    float width = max(fit_ellipse.size.width, fit_ellipse.size.height);
    float height = min(fit_ellipse.size.width, fit_ellipse.size.height);
    float side_ratio = width / height;
    if (side_ratio > rune_target_param.ACTIVE_MAX_SIDE_RATIO || side_ratio < rune_target_param.ACTIVE_MIN_SIDE_RATIO)
        return false;
    float fit_ellipse_area = width * height * CV_PI / 4;
    float area_ratio = contour_area / fit_ellipse_area;
    if (area_ratio > rune_target_param.ACTIVE_MAX_AREA_RATIO || area_ratio < rune_target_param.ACTIVE_MIN_AREA_RATIO)
        return false;
    float perimeter = contour->perimeter();
    float fit_ellipse_perimeter = CV_PI * (3 * (width + height) - sqrt((3 * width + height) * (width + 3 * height)));
    float perimeter_ratio = perimeter / fit_ellipse_perimeter;
    if (perimeter_ratio > rune_target_param.ACTIVE_MAX_PERI_RATIO || perimeter_ratio < rune_target_param.ACTIVE_MIN_PERI_RATIO)
        return false;
    const auto &hull_contour = contour->convexHull();
    float hull_area = contourArea(hull_contour);
    float hull_area_ratio = hull_area / contour_area;
    if (hull_area_ratio > rune_target_param.ACTIVE_MAX_CONVEX_AREA_RATIO)
        return false;
    float hull_perimeter = arcLength(hull_contour, true);
    float hull_perimeter_ratio = hull_perimeter / perimeter;
    hull_perimeter_ratio = hull_perimeter_ratio > 1 ? hull_perimeter_ratio : 1 / hull_perimeter_ratio;
    if (hull_perimeter_ratio > rune_target_param.ACTIVE_MAX_CONVEX_PERI_RATIO)
        return false;
    return true;
}

inline bool checkConcentricity(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, const vector<int> &all_sub_idx, size_t idx, double contour_area)
{
    if (all_sub_idx.empty())
        return false;
    unordered_map<size_t, float> area_map;
    for (auto sub_idx : all_sub_idx)
        area_map[sub_idx] = contours[sub_idx]->area();
    auto [max_area_idx, max_sub_area] = *max_element(area_map.begin(), area_map.end(), [](const auto &lhs, const auto &rhs)
                                                     { return lhs.second < rhs.second; });
    if (max_sub_area > contour_area)
        VC_THROW_ERROR("sub_contour_area > outer_area");
    if (max_sub_area / contour_area < rune_target_param.ACTIVE_MIN_AREA_RATIO_SUB)
        return false;
    return checkEllipse(contours[max_area_idx]);
}

inline bool checkTenRing(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, const vector<int> &all_sub_idx, size_t idx, double contour_area)
{
    float total_area = 0;
    for (auto sub_idx : all_sub_idx)
        total_area += contours[sub_idx]->area();
    return total_area / contour_area <= rune_target_param.ACTIVE_MAX_AREA_RATIO_SUB_TEN_RING;
}

RuneTargetActive_ptr RuneTargetActive::make_feature(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t idx, unordered_set<size_t> &used_contour_idxs)
{
    const auto &contour_outer = contours[idx];
    if (contour_outer->points().size() < 6)
        return nullptr;
    float contour_area = contour_outer->area();
    if (!checkEllipse(contour_outer))
        return nullptr;
    vector<int> all_sub_idx;
    getAllSubContoursIdx(hierarchy, idx, all_sub_idx);
    if (contour_area < 100)
        return nullptr;
    if (!checkConcentricity(contours, hierarchy, all_sub_idx, idx, contour_area) && !checkTenRing(contours, hierarchy, all_sub_idx, idx, contour_area))
        return nullptr;
    used_contour_idxs.insert(idx);
    used_contour_idxs.insert(all_sub_idx.begin(), all_sub_idx.end());
    auto fit_ellipse = contour_outer->fittedEllipse();
    vector<Point2f> corners{fit_ellipse.center};
    return make_shared<RuneTargetActive>(contour_outer, corners);
}

void RuneTargetActive::drawFeature(Mat &image, const DrawConfig_cptr &config) const
{
    auto &image_info = getImageCache();
    auto draw_circle = [&]()
    {
        if (!image_info.isSetCorners() || !image_info.isSetCenter())
            return false;
        auto &center = image_info.getCenter();
        auto radius = image_info.isSetHeight() && image_info.isSetWidth() ? min(image_info.getHeight(), image_info.getWidth()) / 2.0f : rune_target_draw_param.active.point_radius;
        auto &circle_color = rune_target_draw_param.active.color;
        auto circle_thickness = rune_target_draw_param.active.thickness;
        circle(image, center, radius, circle_color, circle_thickness);
        if (radius > 30)
            circle(image, center, radius * 0.5f, circle_color, circle_thickness);
        if (radius > 50)
            circle(image, center, radius * 0.25f, circle_color, circle_thickness);
        if (radius > 100)
            circle(image, center, radius * 0.125f, circle_color, circle_thickness);
        return true;
    };
    auto draw_ellipse = [&]()
    {
        if (!image_info.isSetContours() || image_info.getContours().empty() || image_info.getContours().front()->points().size() < 6)
            return false;
        auto fit_ellipse = image_info.getContours().front()->fittedEllipse();
        auto &circle_color = rune_target_draw_param.active.color;
        auto circle_thickness = rune_target_draw_param.active.thickness;
        ellipse(image, fit_ellipse, circle_color, circle_thickness);
        circle(image, fit_ellipse.center, 2, circle_color, -1);
        return true;
    };
    if (!draw_ellipse())
        draw_circle();
}

RuneTargetActive_ptr RuneTargetActive::make_feature(const PoseNode &target_to_cam)
{
    vector<Point3f> corners_3d{{0, -rune_target_param.RADIUS, 0}, {rune_target_param.RADIUS, 0, 0}, {0, rune_target_param.RADIUS, 0}, {-rune_target_param.RADIUS, 0, 0}};
    vector<Point2f> corners_2d, temp_rune_center;
    projectPoints(corners_3d, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, corners_2d);
    projectPoints(vector<Point3f>{Point3f(0, 0, 0)}, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, temp_rune_center);
    auto result_ptr = make_shared<RuneTargetActive>(temp_rune_center[0], corners_2d);
    if (result_ptr)
        result_ptr->getPoseCache().getPoseNodes()[CoordFrame::CAMERA] = target_to_cam;
    return result_ptr;
}
