#include "vc/feature/rune_target.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_target_active.h"
#include "vc/feature/rune_target_param.h"
#include "vc/camera/camera_param.h"
using namespace std;
using namespace cv;

RuneTarget::RuneTarget(const Contour_cptr contour, const vector<Point2f> corners)
{
    if (contour->points().size() < 3)
        VC_THROW_ERROR("Contour points are less than 3, cannot construct RuneTarget.");
    auto rotate_rect = contour->minAreaRect();
    auto width = max(rotate_rect.size.width, rotate_rect.size.height);
    auto height = min(rotate_rect.size.width, rotate_rect.size.height);
    Point2f center{};
    center = contour->points().size() > 6 ? contour->fittedEllipse().center : contour->center();
    auto &image_info = getImageCache();
    image_info.setContours(vector<Contour_cptr>{contour});
    image_info.setCorners(corners);
    image_info.setCenter(center);
    image_info.setWidth(width);
    image_info.setHeight(height);
}

void RuneTarget::find_active_targets(vector<FeatureNode_ptr> &targets, const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, const unordered_set<size_t> &mask, unordered_map<FeatureNode_cptr, unordered_set<size_t>> &used_contour_idxs)
{
    RuneTargetActive::find(targets, contours, hierarchy, mask, used_contour_idxs);
}

void RuneTarget::find_inactive_targets(vector<FeatureNode_ptr> &targets, const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, const unordered_set<size_t> &mask, unordered_map<FeatureNode_cptr, unordered_set<size_t>> &used_contour_idxs)
{
    RuneTargetInactive::find(targets, contours, hierarchy, mask, used_contour_idxs);
}

auto RuneTarget::getPnpPoints() const -> tuple<vector<Point2f>, vector<Point3f>, vector<float>>
{
    return make_tuple(vector<Point2f>{}, vector<Point3f>{}, vector<float>{});
}

auto RuneTarget::getRelativePnpPoints() const -> tuple<vector<Point2f>, vector<Point3f>, vector<float>>
{
    auto [points_2d, points_3d, weights] = this->getPnpPoints();
    vector<Point3f> relative_points_3d(points_3d.size());
    for (size_t i = 0; i < points_3d.size(); i++)
    {
        Matx31d points_3d_mat(points_3d[i].x, points_3d[i].y, points_3d[i].z);
        Matx31d relative_points_3d_mat = rune_target_param.ROTATION * points_3d_mat + rune_target_param.TRANSLATION;
        relative_points_3d[i] = Point3f(relative_points_3d_mat(0), relative_points_3d_mat(1), relative_points_3d_mat(2));
    }
    return make_tuple(points_2d, relative_points_3d, weights);
}
