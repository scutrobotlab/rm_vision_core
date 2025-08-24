#include "vc/feature/rune_center.h"
#include "vc/feature/rune_center_param.h"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;

RuneCenter::RuneCenter(const Point2f &center_in)
{
    auto contour = ContourWrapper<int>::make_contour({center_in});
    auto center = center_in;
    auto width = rune_center_param.DEFAULT_SIDE;
    auto height = rune_center_param.DEFAULT_SIDE;
    // 更新角点信息
    vector<Point2f> corners(4);
    corners[0] = center + Point2f(-0.5 * width, 0.5 * height);
    corners[1] = center + Point2f(-0.5 * width, -0.5 * height);
    corners[2] = center + Point2f(0.5 * width, -0.5 * height);
    corners[3] = center + Point2f(0.5 * width, 0.5 * height);

    // 初始化构造形状信息
    auto &image_info = getImageCache();
    image_info.setCenter(center);
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCorners(corners);
    image_info.setContours(vector<Contour_cptr>{contour});
}

RuneCenter::RuneCenter(const Contour_cptr &contour, RotatedRect &rotated_rect)
{
    auto min_area_rect = contour->minAreaRect();
    auto center = contour->center();
    auto rrect_size = min_area_rect.size;
    auto width = max(rrect_size.width, rrect_size.height);
    auto height = min(rrect_size.width, rrect_size.height);
    // 更新角点信息
    vector<Point2f> corners(4);
    min_area_rect.points(corners.data());

    auto &image_info = getImageCache();
    image_info.setCenter(center);
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCorners(corners);
    image_info.setContours(vector<Contour_cptr>{contour});
}
/**
 * @brief 神符中心等级向量判断
 *
 * @param[in] hierarchy 所有的等级向量
 * @param[in] idx 指定的等级向量的下标
 * @return 等级结构是否满足要求
 */
inline bool isHierarchyCenter(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t idx)
{
    // h[idx] 必须存在若干并列轮廓，并且无父轮廓
    if ((hierarchy[idx][0] == -1 && hierarchy[idx][1] == -1) || hierarchy[idx][3] != -1)
        return false;
    if (hierarchy[idx][2] == -1)
        return true;
    else if (hierarchy[hierarchy[idx][2]][2] == -1)
    {
        RotatedRect outer = contours[idx]->fittedEllipse();
        Point2f outer_center = outer.center;
        Point2f inner_center = contours[hierarchy[idx][2]]->center();
        auto dis = getDist(inner_center, outer_center);
        auto size = (outer.size.width + outer.size.height) / 2.;
        // 偏移与最大直径的比值
        if (dis / size > rune_center_param.CENTER_CONCENTRICITY_RATIO)
        {
            return true;
        }
        if (contours[hierarchy[idx][2]]->points().size() < 10)
        {
            return true;
        }
        return false;
    }
    return false;
}

/**
 * @brief 获取所有子轮廓
 *
 * @param[in] outer_idx 外轮廓的下标
 * @param[in] contours 所有轮廓
 * @param[in] hierarchy 所有等级向量
 * @param[out] sub_contours 子轮廓
 */
inline void getSubContours(size_t outer_idx, const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, vector<Contour_cptr> &sub_contours)
{
    // 获取所有子轮廓下标
    vector<size_t> sub_contour_idxs{};
    getAllSubContoursIdx(hierarchy, outer_idx, sub_contour_idxs);

    // 获取所有子轮廓
    for (auto idx : sub_contour_idxs)
    {
        sub_contours.push_back(contours[idx]);
    }
}

void RuneCenter::find(std::vector<FeatureNode_ptr> &centers,
                      const std::vector<Contour_cptr> &contours,
                      const std::vector<cv::Vec4i> &hierarchy,
                      const std::unordered_set<size_t> &mask,
                      std::unordered_map<FeatureNode_cptr, unordered_set<size_t>> &used_contour_idxs)
{
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (mask.find(i) != mask.end())
            continue;
        if (isHierarchyCenter(contours, hierarchy, i))
        {
            // 获取所有子轮廓
            vector<Contour_cptr> sub_contours{};
            getSubContours(i, contours, hierarchy, sub_contours);
            auto p_center = RuneCenter::make_feature(contours[i], sub_contours);
            if (p_center != nullptr)
            {
                centers.push_back(p_center);
                used_contour_idxs[p_center] = {i};
            }
        }
    }
}

shared_ptr<RuneCenter> RuneCenter::make_feature(const Point2f &center, bool force)
{
    auto result = make_shared<RuneCenter>(center);
    if (force)
    {
    }

    return result;
}

shared_ptr<RuneCenter> RuneCenter::make_feature(const Contour_cptr &contour, const std::vector<Contour_cptr> &sub_contours)
{
    if (contour->points().size() < 6)
        return nullptr;
    // init
    RotatedRect rotated_rect = contour->fittedEllipse();

    // 1.绝对面积判断
    if (contour->area() < rune_center_param.MIN_AREA)
    {
        return nullptr;
    }
    if (contour->area() > rune_center_param.MAX_AREA)
    {
        return nullptr;
    }

    // 2.比例判断
    float width = max(rotated_rect.size.width, rotated_rect.size.height);
    float height = min(rotated_rect.size.width, rotated_rect.size.height);
    float side_ratio = width / height;
    if (side_ratio > rune_center_param.MAX_SIDE_RATIO)
    {
        return nullptr;
    }
    if (side_ratio < rune_center_param.MIN_SIDE_RATIO)
    {
        return nullptr;
    }

    // 3. 圆形度判断
    float area = contour->area();         // 计算轮廓面积
    float len = contour->perimeter(true); // 计算轮廓周长
    if (len == 0)
    {
        return nullptr;
    }
    float roundness = (4 * CV_PI * area) / (len * len); // 圆形度
    if (roundness < rune_center_param.MIN_ROUNDNESS)
    {
        return nullptr;
    }
    if (roundness > rune_center_param.MAX_ROUNDNESS)
    {
        return nullptr;
    }

    // 4. 父轮廓与子轮廓的面积比例判断
    // 获取子轮廓面积之和
    float total_sub_area = 0;
    for (auto sub_contour : sub_contours)
    {
        total_sub_area += sub_contour->area();
    }
    float sub_area_ratio = total_sub_area / contour->area();
    if (sub_area_ratio > rune_center_param.MAX_SUB_AREA_RATIO && contour->area() > rune_center_param.MIN_AREA_FOR_RATIO)
    {
        return nullptr;
    }

    // 5. 与凸包轮廓的面积比例判断
    float convex_area_ratio = contour->area() / contour->convexArea();
    if (convex_area_ratio < rune_center_param.MIN_CONVEX_AREA_RATIO && contour->area() > rune_center_param.MIN_AREA_FOR_RATIO)
    {
        return nullptr;
    }

    // 6. 最大凹陷面积判断
    float max_defect_area = 0;
    float max_defect_idx = -1;
    vector<Vec4i> defects;
    const auto &hull = contour->convexHullIdx();
    const auto &contour_points = contour->points();
    if (contour_points.size() < 3 || hull.size() < 3)
    {
        return nullptr;
    }
    float defect_area_ratio;
    if (isContourConvex(contour_points))
    {

        convexityDefects(contour_points, hull, defects);
        for (size_t i = 0; i < defects.size(); i++)
        {
            Vec4i &d = defects[i];
            Point start = contour->points()[d[0]];    // 凹陷起点
            Point end = contour->points()[d[1]];      // 凹陷终点
            Point farthest = contour->points()[d[2]]; // 凹陷最远点

            // 计算底边长和深度
            float depth = d[3] / 256.0;              // 深度
            float base_length = getDist(start, end); // 底边长

            // 近似计算凹陷面积
            double defect_area = base_length * depth / 2.0;
            if (defect_area > max_defect_area)
            {
                max_defect_area = defect_area;
                max_defect_idx = i;
            }
        }
        defect_area_ratio = max_defect_area / contour->area();
        if (max_defect_idx != -1 && defect_area_ratio > rune_center_param.MAX_DEFECT_AREA_RATIO && contour->area() > rune_center_param.MIN_AREA_FOR_RATIO)
        {
            return nullptr;
        }
    }
    return make_shared<RuneCenter>(contour, rotated_rect);
}

std::shared_ptr<RuneCenter> RuneCenter::make_feature(const PoseNode &center_to_cam)
{
    vector<Point3f> points_mat_3d{};
    points_mat_3d.push_back(Point3f(0, 0, 0));
    // 重投影
    vector<Point2f> points_2d_reproject{};
    projectPoints(points_mat_3d, center_to_cam.rvec(), center_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, points_2d_reproject);

    RuneCenter_ptr result_ptr = make_feature(points_2d_reproject[0]);
    if (result_ptr)
    {
        auto &pose_info = result_ptr->getPoseCache();
        pose_info.getPoseNodes()[CoordFrame::CAMERA] = center_to_cam;
    }
    return result_ptr;
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> RuneCenter::getPnpPoints() const
{
    vector<Point2f> points_2d;
    vector<Point3f> points_3d;
    vector<float> weights;
    points_2d.push_back(getImageCache().getCenter());
    points_3d.push_back(Point3f(0, 0, 0));

    if (points_2d.size() != points_3d.size())
    {
        VC_THROW_ERROR(" points_2d and points_3d are not equal. (size = %zu, %zu)", points_2d.size(), points_3d.size());
    }
    weights.resize(points_2d.size(), 1.0);

    return make_tuple(points_2d, points_3d, weights);
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> RuneCenter::getRelativePnpPoints() const
{
    auto [points_2d, points_3d, weights] = this->getPnpPoints();
    vector<Point3f> relative_points_3d(points_3d.size());
    for (size_t i = 0; i < points_3d.size(); i++)
    {
        Matx31d points_3d_mat(points_3d[i].x, points_3d[i].y, points_3d[i].z);
        Matx31d relative_points_3d_mat{};
        relative_points_3d_mat = rune_center_param.ROTATION * points_3d_mat + rune_center_param.TRANSLATION;
        relative_points_3d[i] = Point3f(relative_points_3d_mat(0), relative_points_3d_mat(1), relative_points_3d_mat(2));
    }
    return make_tuple(points_2d, relative_points_3d, weights);
}


void RuneCenter::drawFeature(cv::Mat &image, const DrawConfig_cptr &config) const
{
    const auto& image_info = this->getImageCache();
    // 使用默认半径进行绘制
    auto draw_circle = [&]()->bool
    {
        if(!image_info.isSetCenter())
        {
            return false;
        }

        const auto& center = image_info.getCenter();
        auto radius = rune_center_draw_param.default_radius;
        auto color = rune_center_draw_param.color;
        cv::circle(image, center, radius, color, 2);
        return true;
    };

    // 使用轮廓椭圆进行绘制
    auto draw_contour_fit_ellipse = [&]()->bool
    {
        if(!image_info.isSetContours() || image_info.getContours().empty())
        {
            return false;
        }
        const auto& contour = image_info.getContours().front();
        if(contour->points().size() < 6)
        {
            return false;
        }
        auto ellipse = contour->fittedEllipse();
        cv::ellipse(image, ellipse, rune_center_draw_param.color, 2);
        return true;
    };

    do
    {
        if (draw_contour_fit_ellipse())
        {
            break;
        }
        else
        {
            draw_circle();
        }
    }while(0);

}