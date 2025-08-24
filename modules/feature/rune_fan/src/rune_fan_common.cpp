
#include <numeric>
#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_fan_active.h"
#include "vc/feature/rune_fan_inactive.h"
#include "vc/feature/rune_fan_param.h"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;

void RuneFan::find_active_fans(std::vector<FeatureNode_ptr> &fans,
                               const std::vector<Contour_cptr> &contours,
                               const std::vector<cv::Vec4i> &hierarchy,
                               const std::unordered_set<size_t> &mask,
                               std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs)
{
    RuneFanActive::find(fans, contours, hierarchy, mask, used_contour_idxs);
}

void RuneFan::find_inactive_fans(std::vector<FeatureNode_ptr> &fans,
                                 const std::vector<Contour_cptr> &contours,
                                 const std::vector<cv::Vec4i> &hierarchy,
                                 const std::unordered_set<size_t> &mask,
                                 std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs,
                                 const std::vector<FeatureNode_cptr> &inactive_targets)
{
    RuneFanInactive::find(fans, contours, hierarchy, mask, used_contour_idxs, inactive_targets);
}

void RuneFan::find_incomplete_active_fans(std::vector<FeatureNode_ptr> &fans,
                                          const std::vector<Contour_cptr> &contours,
                                          const std::vector<cv::Vec4i> &hierarchy,
                                          const std::unordered_set<size_t> &mask,
                                          const cv::Point2f &rotate_center,
                                          std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs)
{
    RuneFanActive::find_incomplete(fans, contours, hierarchy, mask, rotate_center, used_contour_idxs);
}

auto RuneFan::getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>
{
    // 空实现,不允许被调用
    VC_THROW_ERROR("getPnpPoints() is not implemented in RuneFan class. Please override this method in derived classes.");
    vector<Point2f> points_2d{};
    vector<Point3f> points_3d{};
    vector<float> weights{};
    return make_tuple(points_2d, points_3d, weights);
}

auto RuneFan::getRelativePnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>
{
    auto [points_2d, points_3d, weights] = this->getPnpPoints();
    vector<Point3f> relative_points_3d(points_3d.size());
    for (int i = 0; i < points_3d.size(); i++)
    {
        Matx31d points_3d_mat(points_3d[i].x, points_3d[i].y, points_3d[i].z);
        Matx31d relative_points_3d_mat{};
        if (this->getActiveFlag())
        {
            relative_points_3d_mat = rune_fan_param.ACTIVE_ROTATION * points_3d_mat + rune_fan_param.ACTIVE_TRANSLATION;
        }
        else
        {
            relative_points_3d_mat = rune_fan_param.INACTIVE_ROTATION * points_3d_mat + rune_fan_param.INACTIVE_TRANSLATION;
        }
        relative_points_3d[i] = Point3f(relative_points_3d_mat(0), relative_points_3d_mat(1), relative_points_3d_mat(2));
    }
    return make_tuple(points_2d, relative_points_3d, weights);
}

bool checkPoints(const vector<Point2d> &points)
{
    static const float MAX_X = 5000;
    static const float MAX_Y = 5000;

    for (const auto &point : points)
    {
        if (abs(point.x) > MAX_X || abs(point.y) > MAX_Y)
        {
            return false;
        }
    }
    return true;
}

std::shared_ptr<RuneFan> RuneFan::make_feature(const PoseNode &fan_to_cam, bool is_active)
{
    RuneFan_ptr result_ptr;
    if (is_active)
    {
        vector<Point3d> top_hump_corners{};
        vector<Point3d> bottom_center_hump_corners{};
        vector<Point3d> side_hump_corners{};
        vector<Point3d> bottom_side_hump_corners{};

        top_hump_corners = rune_fan_param.ACTIVE_TOP_3D;
        bottom_center_hump_corners = rune_fan_param.ACTIVE_BOTTOM_CENTER_3D;
        side_hump_corners = rune_fan_param.ACTIVE_SIDE_3D;
        bottom_side_hump_corners = rune_fan_param.ACTIVE_BOTTOM_SIDE_3D;

        vector<Point2d> top_hump_corners_2d{};
        vector<Point2d> bottom_center_hump_corners_2d{};
        vector<Point2d> side_hump_corners_2d{};
        vector<Point2d> bottom_side_hump_corners_2d{};

        projectPoints(top_hump_corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, top_hump_corners_2d);
        projectPoints(bottom_center_hump_corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, bottom_center_hump_corners_2d);
        projectPoints(side_hump_corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, side_hump_corners_2d);
        projectPoints(bottom_side_hump_corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, bottom_side_hump_corners_2d);

        if (checkPoints(top_hump_corners_2d) == false || checkPoints(bottom_center_hump_corners_2d) == false || checkPoints(side_hump_corners_2d) == false || checkPoints(bottom_side_hump_corners_2d) == false)
        {
            return nullptr;
        }

        result_ptr = RuneFanActive::make_feature(top_hump_corners_2d, bottom_center_hump_corners_2d, side_hump_corners_2d, bottom_side_hump_corners_2d);
    }
    else
    {
        vector<Point3d> corners = rune_fan_param.INACTIVE_3D;
        vector<Point2d> corners_2d{};
        projectPoints(corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, corners_2d);

        if (checkPoints(corners_2d) == false)
        {
            return nullptr;
        }

        result_ptr = RuneFanInactive::make_feature(corners_2d[0], corners_2d[1], corners_2d[2], corners_2d[3]);
    }
    if (result_ptr)
    {
        auto &temp_ptr = result_ptr;
        temp_ptr->getPoseCache().getPoseNodes()[CoordFrame::CAMERA] = PoseNode(fan_to_cam);
    }
    return result_ptr;
}
