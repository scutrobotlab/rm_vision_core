#include <opencv2/imgproc.hpp>
#include <unordered_set>
#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/feature/rune_target_param.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/math/geom_utils.hpp"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;

RuneTargetInactive::RuneTargetInactive(const Contour_cptr contour, const vector<Point2f> corners, vector<RuneTargetGap> gaps)
    : RuneTarget(contour, corners)
{
    setActiveFlag(false);
    setGaps(gaps);
}

RuneTargetInactive::RuneTargetInactive(const Point2f center, const vector<Point2f> corners)
{
    vector<Point2f> temp_contour(corners.begin(), corners.end());
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
    setActiveFlag(false);
    auto &image_info = getImageCache();
    image_info.setCenter(center);
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCorners(corners);
    image_info.setContours(vector<Contour_cptr>{contour});
}

inline bool isHierarchyInactiveTarget(const vector<Vec4i> &hierarchy, size_t idx)
{
    if (hierarchy[idx][3] != -1)
        return false;
    if (hierarchy[idx][2] == -1)
        return false;
    int child_num = 1;
    int front_child_idx = hierarchy[hierarchy[idx][2]][1];
    while (front_child_idx != -1)
    {
        child_num++;
        front_child_idx = hierarchy[front_child_idx][1];
    }
    int back_child_idx = hierarchy[hierarchy[idx][2]][0];
    while (back_child_idx != -1)
    {
        child_num++;
        back_child_idx = hierarchy[back_child_idx][0];
    }
    return child_num >= 4;
}

inline bool isContourInactiveTarget(const vector<Contour_cptr> &contours, const int &outer_idx, const unordered_set<size_t> &sub_idxs)
{
    if (contours[outer_idx]->points().size() < 6)
        return false;
    float contour_area = contours[outer_idx]->area();
    if (contour_area < rune_target_param.INACTIVE_MIN_AREA)
        return false;
    // 利用长宽比、面积比、周长比进行筛选
    auto fit_ellipse = contours[outer_idx]->fittedEllipse();
    float width = max(fit_ellipse.size.width, fit_ellipse.size.height);
    float height = min(fit_ellipse.size.width, fit_ellipse.size.height);
    float side_ratio = width / height;
    if (side_ratio > rune_target_param.INACTIVE_MAX_SIDE_RATIO || side_ratio < rune_target_param.INACTIVE_MIN_SIDE_RATIO)
        return false;

    float fit_ellipse_area = width * height * CV_PI / 4;
    float area_ratio = contour_area / fit_ellipse_area;
    if (area_ratio > rune_target_param.INACTIVE_MAX_AREA_RATIO || area_ratio < rune_target_param.INACTIVE_MIN_AREA_RATIO)
        return false;

    float perimeter = contours[outer_idx]->perimeter();
    float fit_ellipse_perimeter = CV_PI * (3 * (width + height) - sqrt((3 * width + height) * (width + 3 * height)));
    float perimeter_ratio = perimeter / fit_ellipse_perimeter;
    if (perimeter_ratio > rune_target_param.INACTIVE_MAX_PERI_RATIO || perimeter_ratio < rune_target_param.INACTIVE_MIN_PERI_RATIO)
        return false;
        
    return true;
}

inline RotatedRect getRatioEllipse(const RotatedRect &ellipse, float ratio)
{
    return RotatedRect(ellipse.center, Size2f(ellipse.size.width * ratio, ellipse.size.height * ratio), ellipse.angle);
}

inline bool isPointInEllipse(const Point2f &point, const RotatedRect &ellipse)
{
    Point2f translated_point = point - ellipse.center;
    float cos_angle = cos(-ellipse.angle * CV_PI / 180.0);
    float sin_angle = sin(-ellipse.angle * CV_PI / 180.0);
    Point2f rotated_point(translated_point.x * cos_angle - translated_point.y * sin_angle,
                          translated_point.x * sin_angle + translated_point.y * cos_angle);
    float x_radius = ellipse.size.width / 2.0;
    float y_radius = ellipse.size.height / 2.0;
    return (rotated_point.x * x_radius) * (rotated_point.x * x_radius) + (rotated_point.y * y_radius) * (rotated_point.y * y_radius) <= 1.0;
}

// 获取椭圆矫正矩阵
inline Matx33f getEllipseCorrectionMat(const RotatedRect &ellipse, float &radius)
{
    Mat temp_R = getRotationMatrix2D(ellipse.center, ellipse.angle, 1.0);
    Matx33f R(temp_R.at<double>(0, 0), temp_R.at<double>(0, 1), temp_R.at<double>(0, 2),
              temp_R.at<double>(1, 0), temp_R.at<double>(1, 1), temp_R.at<double>(1, 2),
              0, 0, 1);
    Matx33f R_inv = R.inv();
    Matx33f T(1, 0, -ellipse.center.x, 0, 1, -ellipse.center.y, 0, 0, 1);
    float len = (ellipse.size.width + ellipse.size.height) * 2.0;
    Matx33f S(len / ellipse.size.width, 0, 0, 0, len / ellipse.size.height, 0, 0, 0, 1);
    radius = len / 2.0;
    Matx33f T_inv = T.inv();
    return R_inv * T_inv * S * T * R;
}

inline unordered_set<size_t> getAllGapsIdx(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, const int outer_idx, const unordered_set<size_t> &all_sub_idx)
{
    unordered_set<size_t> pending_idx(all_sub_idx.begin(), all_sub_idx.end());
    for (auto &sub_idx : all_sub_idx)
    {
        if (hierarchy[sub_idx][3] == -1)
            pending_idx.erase(sub_idx);
        if (hierarchy[sub_idx][3] != outer_idx)
            pending_idx.erase(sub_idx);
        if (contours[sub_idx]->points().size() < 10)
            pending_idx.erase(sub_idx);
    }
    auto outer_fit_ellipse = contours[outer_idx]->fittedEllipse();
    Point2f outer_center = outer_fit_ellipse.center;
    for (auto &sub_idx : all_sub_idx)
    {
        if (pending_idx.find(sub_idx) == pending_idx.end())
            continue;
        if (pointPolygonTest(contours[sub_idx]->points(), outer_center, false) > 0)
            pending_idx.erase(sub_idx);
    }
    unordered_map<size_t, float> area_map;
    float current_area = contours[outer_idx]->area();
    for (auto &sub_idx : pending_idx)
        area_map[sub_idx] = contours[sub_idx]->area();
    for (auto &[sub_idx, area] : area_map)
        if (area / current_area > rune_target_param.GAP_MAX_AREA_RATIO || area / current_area < rune_target_param.GAP_MIN_AREA_RATIO)
            pending_idx.erase(sub_idx);
    for (auto &sub_idx : all_sub_idx)
    {
        if (pending_idx.find(sub_idx) == pending_idx.end())
            continue;
        auto fit_ellipse = contours[sub_idx]->fittedEllipse();
        float width = max(fit_ellipse.size.width, fit_ellipse.size.height);
        float height = min(fit_ellipse.size.width, fit_ellipse.size.height);
        float side_ratio = width / height;
        if (side_ratio > rune_target_param.GAP_MAX_SIDE_RATIO || side_ratio < rune_target_param.GAP_MIN_SIDE_RATIO)
            pending_idx.erase(sub_idx);
    }
    return pending_idx;
}

// 获取轮廓最左和最右点的下标
inline tuple<int, int> getLeftRightIdx(const vector<Point2f> &contour, const Point2f &center)
{
    size_t left_idx = 0, right_idx = 0;
    Point2f left_v = contour[left_idx] - center, right_v = contour[right_idx] - center;
    for (size_t i = 0; i < contour.size(); i++)
    {
        Point2f current_v = contour[i] - center;
        if (left_v.cross(current_v) < 0)
        {
            left_v = current_v;
            left_idx = i;
        }
        if (right_v.cross(current_v) > 0)
        {
            right_v = current_v;
            right_idx = i;
        }
    }
    return make_tuple(left_idx, right_idx);
}

// 创建缺口
inline bool make_gap(const Contour_cptr &contour, const RotatedRect &outer_ellipse, RuneTargetGap &result_gap)
{
    float correction_radius = 0;
    Matx33f correction_mat = getEllipseCorrectionMat(outer_ellipse, correction_radius);
    Mat contour_mat(3, contour->points().size(), CV_32F);
    float *x_ptr = contour_mat.ptr<float>(0), *y_ptr = contour_mat.ptr<float>(1), *one_ptr = contour_mat.ptr<float>(2);
    const Point *gap_points = contour->points().data();
    for (size_t i = 0; i < static_cast<size_t>(contour_mat.cols); i++)
    {
        *x_ptr++ = gap_points->x;
        *y_ptr++ = gap_points->y;
        *one_ptr++ = 1;
        gap_points++;
    }
    Mat correction_contour_mat = correction_mat * contour_mat;
    vector<Point2f> correction_contour(contour_mat.cols);
    float *correction_x_ptr = correction_contour_mat.ptr<float>(0), *correction_y_ptr = correction_contour_mat.ptr<float>(1);
    Point2f *correction_points = correction_contour.data();
    for (size_t i = 0; i < static_cast<size_t>(correction_contour_mat.cols); i++)
    {
        correction_points->x = *correction_x_ptr++;
        correction_points->y = *correction_y_ptr++;
        correction_points++;
    }
    auto correction_ellipse = fitEllipse(correction_contour);
    float center_distance = getDist(correction_ellipse.center, outer_ellipse.center);
    if (center_distance > correction_radius * rune_target_param.GAP_MAX_DISTANCE_RATIO || center_distance < correction_radius * rune_target_param.GAP_MIN_DISTANCE_RATIO)
        return false;

    // 获取左、右角点
    auto [left_idx, right_idx] = getLeftRightIdx(correction_contour, outer_ellipse.center);
    Point2f left_corner = getUnitVector(correction_contour[left_idx] - outer_ellipse.center) * rune_target_param.GAP_CIRCLE_RADIUS_RATIO * correction_radius + outer_ellipse.center;
    Point2f right_corner = getUnitVector(correction_contour[right_idx] - outer_ellipse.center) * rune_target_param.GAP_CIRCLE_RADIUS_RATIO * correction_radius + outer_ellipse.center;
    Point2f gap_center = getUnitVector(correction_ellipse.center - outer_ellipse.center) * rune_target_param.GAP_CIRCLE_RADIUS_RATIO * correction_radius + outer_ellipse.center;
    float delta_angle = getVectorMinAngle(left_corner - outer_ellipse.center, right_corner - outer_ellipse.center, DEG);
    if (delta_angle > 100 || delta_angle < 60)
        return false;

    // 获取缺口方向
    Point2f gap_direction = getUnitVector(right_corner - left_corner), center_to_gap = getUnitVector(gap_center - outer_ellipse.center);
    float direction_angle = getVectorMinAngle(gap_direction, center_to_gap, DEG);
    static const float direction_angle_threshold_low = 70, direction_angle_threshold_high = 110;
    if (direction_angle < direction_angle_threshold_low || direction_angle > direction_angle_threshold_high)
        return false;
    
    // 进行反变换
    Matx33f correction_mat_inv = correction_mat.inv();
    auto get_inv_point = [&correction_mat_inv](const Point2f &point) -> Point2f
    {Matx31f point_mat(point.x,point.y,1);Matx31f result_mat=correction_mat_inv*point_mat;return Point2f(result_mat(0),result_mat(1)); };
    result_gap.left_corner = get_inv_point(left_corner);
    result_gap.right_corner = get_inv_point(right_corner);
    result_gap.center = get_inv_point(gap_center);
    return true;
}

inline bool filterGaps(vector<RuneTargetGap> &filter_gaps, const vector<RuneTargetGap> &all_gaps)
{
    filter_gaps = all_gaps;
    return true;
}

inline bool findGaps(vector<RuneTargetGap> &gaps, const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t outer_idx, const unordered_set<size_t> &all_sub_idx)
{
    unordered_set<size_t> pending_idx = getAllGapsIdx(contours, hierarchy, outer_idx, all_sub_idx);
    if (pending_idx.empty())
        return false;
    auto outer_ellipse = contours[outer_idx]->fittedEllipse();
    vector<RuneTargetGap> all_gaps;
    for (auto &gap_idx : pending_idx)
    {
        RuneTargetGap gap;
        if (!make_gap(contours[gap_idx], outer_ellipse, gap))
            continue;
        all_gaps.push_back(gap);
    }
    vector<RuneTargetGap> filter_gaps;
    if (!filterGaps(filter_gaps, all_gaps))
        return false;
    if (filter_gaps.empty() || filter_gaps.size() > 4)
        return false;
    gaps = filter_gaps;
    return true;
}

auto RuneTargetInactive::make_feature(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t idx, unordered_set<size_t> &used_contour_idxs) -> RuneTargetInactive_ptr
{
    unordered_set<size_t> sub_contours_idx{};
    getAllSubContoursIdx(hierarchy, idx, sub_contours_idx);
    used_contour_idxs.insert(idx);
    used_contour_idxs.insert(sub_contours_idx.begin(), sub_contours_idx.end());
    if (!isContourInactiveTarget(contours, idx, sub_contours_idx))
        return nullptr;
    vector<RuneTargetGap> gaps;
    if (!findGaps(gaps, contours, hierarchy, idx, sub_contours_idx))
        return nullptr;
    auto &outer_contour = contours[idx];
    vector<Point2f> corners{outer_contour->fittedEllipse().center};
    return make_shared<RuneTargetInactive>(outer_contour, corners, gaps);
}

void RuneTargetInactive::find(vector<FeatureNode_ptr> &targets, const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, const unordered_set<size_t> &mask, unordered_map<FeatureNode_cptr, unordered_set<size_t>> &used_contour_idxs)
{
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (mask.find(i) != mask.end())
            continue;
        if (isHierarchyInactiveTarget(hierarchy, i))
        {
            unordered_set<size_t> temp_used_contour_idxs{};
            auto p_target = make_feature(contours, hierarchy, i, temp_used_contour_idxs);
            if (p_target)
            {
                targets.push_back(p_target);
                used_contour_idxs[p_target] = temp_used_contour_idxs;
            }
        }
    }
}

bool RuneTargetInactive::correct(const Point2f &rotate_center)
{
    if (isnan(rotate_center.x) || isnan(rotate_center.y))
        return false;
    if (getActiveFlag())
    {
    }
    else
    {
        if (!correctCorners(rotate_center))
            return false;
        if (!correctDirection())
            return false;
    }
    return true;
}

bool RuneTargetInactive::sortedGap(const Point2f &rune_center)
{
    if (getActiveFlag())
    {
        VC_THROW_ERROR("The target is active.");
        return false;
    }
    if (getGapSortedFlag())
    {
        VC_THROW_ERROR("The gap corners have been set.");
        return false;
    }
    auto &gaps = getGaps();
    if (gaps.empty() || gaps.size() > 4)
        return false;
    auto center = getImageCache().getCenter();
    Point2f center_to_RuneCenter = getUnitVector(rune_center - center);
    RuneTargetGap *left_top_gap = nullptr, *right_top_gap = nullptr, *right_bottom_gap = nullptr, *left_bottom_gap = nullptr;
    for (auto &gap : gaps)
    {
        Point2f center_to_gap = getUnitVector(gap.center - center);
        float dot = center_to_RuneCenter.x * center_to_gap.x + center_to_RuneCenter.y * center_to_gap.y;
        float cross = center_to_RuneCenter.cross(center_to_gap);
        if (dot < 0)
        {
            if (cross > 0 && !left_top_gap)
                left_top_gap = &gap;
            else if (cross <= 0 && !right_top_gap)
                right_top_gap = &gap;
        }
        else
        {
            if (cross > 0 && !left_bottom_gap)
                left_bottom_gap = &gap;
            else if (cross <= 0 && !right_bottom_gap)
                right_bottom_gap = &gap;
        }
    }
    const static RuneTargetGap vacancy_gap = {VACANCY_POINT, VACANCY_POINT, VACANCY_POINT, false};
    vector<RuneTargetGap> sorted_gaps(4);
    sorted_gaps[0] = left_top_gap ? *left_top_gap : vacancy_gap;
    sorted_gaps[1] = right_top_gap ? *right_top_gap : vacancy_gap;
    sorted_gaps[2] = right_bottom_gap ? *right_bottom_gap : vacancy_gap;
    sorted_gaps[3] = left_bottom_gap ? *left_bottom_gap : vacancy_gap;
    setLeftTopGap(&sorted_gaps[0]);
    setRightTopGap(&sorted_gaps[1]);
    setRightBottomGap(&sorted_gaps[2]);
    setLeftBottomGap(&sorted_gaps[3]);
    setGaps(sorted_gaps);
    setGapSortedFlag(true);
    return true;
}

bool RuneTargetInactive::calcGapCorners(const Point2f &rune_center)
{
    if (getActiveFlag())
    {
        VC_THROW_ERROR("The target is active.");
        return false;
    }
    if (!getGapSortedFlag())
    {
        VC_THROW_ERROR("The gap is not sorted.");
    }
    if (getGaps().size() != 4)
    {
        VC_THROW_ERROR("The gap size is not 4.");
        return false;
    }
    auto &gap = getGaps();
    auto &gap_corners = getGapCorners();
    auto &target_corners = getImageCache().getCorners();
    gap_corners.clear();
    for (size_t i = 0; i < gap.size(); i++)
    {
        RuneTargetGap *current_gap = &gap[i];
        RuneTargetGap *next_gap = &gap[(i + 1) % 4];
        gap_corners.push_back(current_gap->is_valid ? current_gap->center : VACANCY_POINT);
        gap_corners.push_back(current_gap->is_valid && next_gap->is_valid ? (current_gap->right_corner + next_gap->left_corner) / 2.0 : VACANCY_POINT);
    }
    if (gap_corners.size() != 8)
    {
        VC_THROW_ERROR("The gap corners size is not 8.");
        return false;
    }
    for (auto &gap_corner : gap_corners)
    {
        if (gap_corner != VACANCY_POINT)
            target_corners.push_back(gap_corner);
    }
    return true;
}

bool RuneTargetInactive::correctCorners(const Point2f &rune_center)
{
    if (getActiveFlag())
    {
        VC_THROW_ERROR("The target is active.");
        return false;
    }
    if (!isSetDirection())
        setDirection(getUnitVector(rune_center - getImageCache().getCenter()));
    if (!getGapSortedFlag() && !sortedGap(rune_center))
        return false;
    if (!calcGapCorners(rune_center))
        return false;
    return true;
}

bool RuneTargetInactive::correctDirection()
{
    if (getActiveFlag())
    {
        VC_THROW_ERROR("The target is active.");
        return false;
    }
    if (!getGapSortedFlag())
        return false;
    Point2f direction_left{}, direction_right{}, direction_total{};
    if (getLeftTopGap() && getLeftBottomGap())
        direction_left = getUnitVector(getLeftBottomGap()->center - getLeftTopGap()->center);
    if (getRightTopGap() && getRightBottomGap())
        direction_right = getUnitVector(getRightBottomGap()->center - getRightTopGap()->center);
    if (direction_left == Point2f(0, 0) && direction_right == Point2f(0, 0))
        return false;
    direction_total = (direction_left == Point2f(0, 0) ? direction_right : (direction_right == Point2f(0, 0) ? direction_left : (direction_left + direction_right) / 2.0));
    setDirection(direction_total);
    return true;
}

auto RuneTargetInactive::getPnpPoints() const -> tuple<vector<Point2f>, vector<Point3f>, vector<float>>
{
    vector<Point2f> pnp_points_2d{getImageCache().getCenter()};
    vector<Point3f> pnp_points_3d{Point3f(0, 0, 0)};
    vector<float> pnp_points_weight{1.0};
    if (!getGapSortedFlag())
        return make_tuple(pnp_points_2d, pnp_points_3d, pnp_points_weight);
    auto &gap_corners = getGapCorners();
    if (gap_corners.size() != 8)
        VC_THROW_ERROR("The gap corners size is not 8.");
    for (size_t i = 0; i < gap_corners.size(); i++)
    {
        if (gap_corners[i] != VACANCY_POINT)
        {
            pnp_points_2d.push_back(gap_corners[i]);
            pnp_points_3d.push_back(rune_target_param.GAP_3D[i]);
            pnp_points_weight.push_back(1.0);
        }
    }
    return make_tuple(pnp_points_2d, pnp_points_3d, pnp_points_weight);
}

void RuneTargetInactive::drawFeature(Mat &image, const DrawConfig_cptr &config) const
{
    auto draw_circle = [&]() -> bool
    {
        const auto &image_info = getImageCache();
        if (!image_info.isSetCorners())
            return false;
        const auto &center = image_info.getCenter();
        auto default_radius = rune_target_draw_param.inactive.point_radius;
        auto circle_color = rune_target_draw_param.inactive.color;
        auto circle_thickness = rune_target_draw_param.inactive.thickness;
        float radius = image_info.isSetHeight() && image_info.isSetWidth()
                           ? min(image_info.getHeight(), image_info.getWidth()) / 2.0f
                           : default_radius;
        circle(image, center, radius, circle_color, circle_thickness);
        if (radius > 30)
            circle(image, center, radius * 0.5f, circle_color, circle_thickness);
        if (radius > 50)
            circle(image, center, radius * 0.25f, circle_color, circle_thickness);
        if (radius > 100)
            circle(image, center, radius * 0.125f, circle_color, circle_thickness);
        return true;
    };

    auto draw_ellipse = [&]() -> bool
    {
        const auto &image_info = getImageCache();
        if (!image_info.isSetContours())
            return false;
        const auto &contours = image_info.getContours();
        if (contours.empty() || contours.front()->points().size() < 6)
            return false;
        auto fit_ellipse = contours.front()->fittedEllipse();
        auto circle_color = rune_target_draw_param.inactive.color;
        auto circle_thickness = rune_target_draw_param.inactive.thickness;
        ellipse(image, fit_ellipse, circle_color, circle_thickness);
        circle(image, fit_ellipse.center, 2, circle_color, -1);
        return true;
    };

    auto draw_corners = [&]() -> bool
    {
        const auto &image_info = getImageCache();
        if (!image_info.isSetCorners())
            return false;
        const auto &corners = image_info.getCorners();
        for (int i = 0; i < (int)corners.size(); i++)
        {
            auto color = rune_target_draw_param.inactive.color;
            auto thickness = rune_target_draw_param.inactive.thickness;
            auto point_radius = rune_target_draw_param.inactive.point_radius;
            circle(image, corners[i], point_radius, color, thickness, LINE_AA);
        }
        for (int i = 0; i < (int)corners.size(); i++)
        {
            auto font_scale = rune_target_draw_param.inactive.font_scale;
            auto font_thickness = rune_target_draw_param.inactive.font_thickness;
            auto font_color = rune_target_draw_param.inactive.font_color;
            putText(image, to_string(i), corners[i], FONT_HERSHEY_SIMPLEX, font_scale, font_color, font_thickness, LINE_AA);
        }
        return true;
    };

    do
    {
        if (!draw_ellipse())
            draw_circle();
        draw_corners();

    } while (0);
}

RuneTargetInactive_ptr RuneTargetInactive::make_feature(const PoseNode &target_to_cam)
{
    vector<Point3f> corners_3d;
    for (const auto &corner : rune_target_param.GAP_3D)
        corners_3d.emplace_back(corner);
    vector<Point2f> corners_2d;
    projectPoints(corners_3d, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, corners_2d);
    vector<Point2f> temp_rune_center;
    projectPoints(vector<Point3f>{Point3f(0, 0, 0)}, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, temp_rune_center);
    auto result_ptr = make_shared<RuneTargetInactive>(temp_rune_center[0], corners_2d);
    if (result_ptr)
    {
        auto &pose_info = result_ptr->getPoseCache();
        pose_info.getPoseNodes()[CoordFrame::CAMERA] = target_to_cam;
    }
    return result_ptr;
}
