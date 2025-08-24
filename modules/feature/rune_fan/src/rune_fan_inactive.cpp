#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/feature/rune_fan_inactive.h"
#include "vc/feature/rune_fan_param.h"



using namespace std;
using namespace cv;

inline bool isHierarchyInactiveFan(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t idx)
{
    return hierarchy[idx][3] == -1;
}

inline double calculateProjectionRatio(const RotatedRect &rect1, const RotatedRect &rect2)
{
    const RotatedRect &largeRect = rect1.size.area() > rect2.size.area() ? rect1 : rect2;
    const RotatedRect &smallRect = rect1.size.area() > rect2.size.area() ? rect2 : rect1;

    Point2f largePoints[4], smallPoints[4];
    largeRect.points(largePoints);
    smallRect.points(smallPoints);

    struct Edge
    {
        Point2f start, end, dir;
        double len;
    };
    Edge edges[2];
    for (int i = 0; i < 2; ++i)
    {
        edges[i].start = largePoints[i];
        edges[i].end = largePoints[(i + 1) % 4];
        edges[i].dir = edges[i].end - edges[i].start;
        edges[i].len = norm(edges[i].dir);
        edges[i].dir /= edges[i].len;
    }

    vector<double> proj_x, proj_y;
    for (int i = 0; i < 4; ++i)
    {
        proj_x.push_back((smallPoints[i] - edges[0].start).dot(edges[0].dir));
        proj_y.push_back((smallPoints[i] - edges[1].start).dot(edges[1].dir));
    }

    auto range = [](const vector<double> &v)
    { return make_pair(*min_element(v.begin(), v.end()), *max_element(v.begin(), v.end())); };
    auto [x_min, x_max] = range(proj_x);
    auto [y_min, y_max] = range(proj_y);
    double x_len = edges[0].len, y_len = edges[1].len;
    double x_int = max(0.0, min(x_max, x_len) - max(x_min, 0.0));
    double y_int = max(0.0, min(y_max, y_len) - max(y_min, 0.0));
    return x_int / x_len > y_int / y_len ? (x_max - x_min) / x_len : y_int / y_len;
}

inline void filterFanContours(const std::vector<Contour_cptr> &in_contours, std::vector<Contour_cptr> &out_contours, vector<FeatureNode_ptr> &inactive_targets)
{
    if (in_contours.size() < 2 || inactive_targets.empty())
        return;

    FeatureNode_ptr ref_target = nullptr;
    if (inactive_targets.size() > 1) // 多个靶心，选择最近的那个
    {
        unordered_map<Contour_cptr, double> contour_weights{};
        double area_sum = 0;
        for (auto &contour : in_contours)
            area_sum += contour->area();

        if (area_sum == 0)
            return;

        for (auto &contour : in_contours)
        {
            contour_weights[contour] = contour->area() / area_sum;
        }

        Point2f all_contours_center{0, 0};
        for (auto &contour : in_contours)
            all_contours_center += static_cast<Point2f>(contour->center()) * contour_weights[contour];

        auto near_target = *min_element(inactive_targets.begin(), inactive_targets.end(), [&](const FeatureNode_ptr &a, const FeatureNode_ptr &b)
                                        { return norm(a->getImageCache().getCenter() - all_contours_center) < norm(b->getImageCache().getCenter() - all_contours_center); });
    }
    else
    {
        ref_target = inactive_targets.front();
    }

    auto far_contour = *max_element(in_contours.begin(), in_contours.end(), [&](const Contour_cptr &a, const Contour_cptr &b)
                                    { return norm(static_cast<Point2f>(a->center()) - ref_target->getImageCache().getCenter()) < norm(static_cast<Point2f>(b->center()) - ref_target->getImageCache().getCenter()); });

    out_contours = in_contours;
    out_contours.erase(remove_if(out_contours.begin(), out_contours.end(), [&](const Contour_cptr &contour)
                                 { return contour == far_contour; }),
                       out_contours.end());
}

void RuneFanInactive::find(std::vector<FeatureNode_ptr> &fans,
                 const std::vector<Contour_cptr> &contours,
                 const std::vector<cv::Vec4i> &hierarchy,
                 const std::unordered_set<size_t> &mask,
                 std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs,
                 const std::vector<FeatureNode_cptr> &inactive_targets)
{
    if (contours.empty() || hierarchy.empty())
    {
        VC_THROW_ERROR("The contours or hierarchy is empty. to find inactive fans.");
    }
    if (contours.size() != hierarchy.size())
    {
        VC_THROW_ERROR("The contours size is not equal to hierarchy size. to find inactive fans.");
    }
    fans.clear();

    // 获取查找范围集合
    vector<size_t> find_idxs{};
    for (auto i = 0; i < contours.size(); i++)
    {
        if (mask.find(i) != mask.end())
            continue;
        if (isHierarchyInactiveFan(contours, hierarchy, i) == false)
            continue;
        find_idxs.push_back(i);
    }

    vector<vector<size_t>> contours_group{};
    for (const auto &idx : find_idxs)
    {
        contours_group.emplace_back(vector<size_t>(1, idx)); // 初始化每个轮廓为一个组
    }
    for (auto &group : contours_group)
    {
        vector<Contour_cptr> temp_contours{};
        for (auto &idx : group)
            temp_contours.push_back(contours[idx]);
        auto p_fan = RuneFanInactive::make_feature(temp_contours);
        if (p_fan)
        {
            fans.push_back(p_fan);
            used_contour_idxs.insert({p_fan, {group.begin(), group.end()}});
        }
    }
}


RuneFanInactive_ptr RuneFanInactive::make_feature(const vector<Contour_cptr> &contours)
{
    if (contours.empty())
        return nullptr;

    vector<Point> temp_contour{};
    for (auto &contour : contours)
        temp_contour.insert(temp_contour.end(), contour->points().begin(), contour->points().end());
    vector<Point> hull_contour_temp{};
    convexHull(temp_contour, hull_contour_temp);
    Contour_cptr hull_contour = ContourWrapper<int>::make_contour(hull_contour_temp);
    double hull_area = hull_contour->area();
    RotatedRect rotated_rect = hull_contour->minAreaRect();

    double rect_area = rotated_rect.size.area();
    if (hull_area < rune_fan_param.INACTIVE_MIN_AREA)
    {
        return nullptr;
    }
    if (hull_area > rune_fan_param.INACTIVE_MAX_AREA)
    {
        return nullptr;
    }

    double area_ratio = hull_area / rotated_rect.size.area();
    if (area_ratio < rune_fan_param.INACTIVE_MIN_AREA_RATIO)
    {
        return nullptr;
    }

    double width = max(rotated_rect.size.width, rotated_rect.size.height);
    double height = min(rotated_rect.size.width, rotated_rect.size.height);
    double side_ratio = width / height;
    if (side_ratio < rune_fan_param.INACTIVE_MIN_SIDE_RATIO)
    {
        return nullptr;
    }
    if (side_ratio > rune_fan_param.INACTIVE_MAX_SIDE_RATIO)
    {
        return nullptr;
    }

    return make_shared<RuneFanInactive>(hull_contour, contours, rotated_rect);
}

RuneFanInactive_ptr RuneFanInactive::make_feature(const Point2f &top_left,
                                                      const Point2f &top_right,
                                                      const Point2f &bottom_right,
                                                      const Point2f &bottom_left)
{
    // 若点发生重合，返回空指针
    if (top_left == top_right || top_left == bottom_right || top_left == bottom_left || top_right == bottom_right || top_right == bottom_left || bottom_right == bottom_left)
        return nullptr;

    return make_shared<RuneFanInactive>(top_left, top_right, bottom_right, bottom_left);
}

RuneFanInactive::RuneFanInactive(const Contour_cptr hull_contour, const vector<Contour_cptr> &arrow_contours, const RotatedRect &rotated_rect)
{
    auto width = min(rotated_rect.size.width, rotated_rect.size.height);
    auto height = max(rotated_rect.size.width, rotated_rect.size.height);
    auto center = rotated_rect.center;
    vector<Point2f> corners(4);
    rotated_rect.points(corners.data());

    // 取最小外接矩形的长边作为方向
    Point2f direction_temp{};
    if (getDist(corners[0], corners[1]) > getDist(corners[0], corners[3]))
        direction_temp = corners[0] - corners[1];
    else
        direction_temp = corners[0] - corners[3];

    setArrowContours(arrow_contours);
    setActiveFlag(false);
    setRotatedRect(rotated_rect);

    auto& image_info = getImageCache();
    image_info.setContours(vector<Contour_cptr>{hull_contour});
    image_info.setCorners(corners);
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCenter(center);
    image_info.setDirection(getUnitVector(direction_temp)); // 扇叶的方向指向神符中心
}

RuneFanInactive::RuneFanInactive(const Point2f &top_left,
                                 const Point2f &top_right,
                                 const Point2f &bottom_right,
                                 const Point2f &bottom_left)
{
    Point2f top_center = (top_left + top_right) / 2;
    Point2f bottom_center = (bottom_left + bottom_right) / 2;
    Point2f left_center = (top_left + bottom_left) / 2;
    Point2f right_center = (top_right + bottom_right) / 2;

    auto center = (top_center + bottom_center) / 2;
    auto contour = ContourWrapper<int>::make_contour({static_cast<Point>(top_left), static_cast<Point>(top_right), static_cast<Point>(bottom_right), static_cast<Point>(bottom_left)});
    auto corners = {top_left, top_right, bottom_right, bottom_left};
    auto width = getDist(left_center, right_center);
    auto height = getDist(top_center, bottom_center);

    setRotatedRect(contour->minAreaRect());
    setActiveFlag(false);
    auto& image_info = getImageCache();
    image_info.setContours(vector<Contour_cptr>{contour});
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCenter(center);
    image_info.setCorners(corners);
    image_info.setDirection(getUnitVector(bottom_center - top_center)); // 扇叶的方向指向神符中心
}

bool RuneFanInactive::correctDirection(FeatureNode_ptr &fan, const cv::Point2f &correct_center)
{
    auto rune_fan = RuneFanInactive::cast(fan);
    if (rune_fan == nullptr)
        return false;
    if (rune_fan->getActiveFlag())
        return false;
    if (rune_fan->getImageCache().getDirection() == cv::Point2f(0, 0))
        return false;
    if (correct_center == cv::Point2f(0, 0))
        return false;
    vector<Point2f> rect_points(4);
    rune_fan->getRotatedRect().points(rect_points.data());
    Point2f direction{};
    if (getDist(rect_points[0], rect_points[1]) > getDist(rect_points[0], rect_points[3]))
        direction = rect_points[0] - rect_points[1];
    else
        direction = rect_points[0] - rect_points[3];

    // 参考方向
    Point2f reference_direction = getUnitVector(correct_center - rune_fan->getImageCache().getCenter());
    if (direction.dot(reference_direction) < 0)
        direction = -direction;
    rune_fan->getImageCache().setDirection(getUnitVector(direction));

    return true;
}


bool RuneFanInactive::correctCorners(FeatureNode_ptr &fan)
{
    auto rune_fan = RuneFanInactive::cast(fan);
    if (rune_fan == nullptr)
        return false;
    if (rune_fan->getActiveFlag())
        return false;
    if (rune_fan->getImageCache().getDirection() == cv::Point2f(0, 0))
        return false;

    vector<Point2f> rect_points(4);
    rune_fan->getRotatedRect().points(rect_points.data());
    vector<Point2f> temp_corners(rect_points.begin(), rect_points.end());
    Point2f dirction = rune_fan->getImageCache().getDirection();
    Point2f center = rune_fan->getRotatedRect().center;
    // 按照在方向上的投影排序
    sort(temp_corners.begin(), temp_corners.end(), [&](Point2f p1, Point2f p2)
         {
        Point2f v1 = p1 - center;
        Point2f v2 = p2 - center;
        return v1.dot(dirction) < v2.dot(dirction); });
    Point2f v0 = temp_corners[0] - center;
    Point2f v1 = temp_corners[1] - center;
    Point2f v2 = temp_corners[2] - center;
    Point2f v3 = temp_corners[3] - center;
    if (v0.cross(v1) < 0)
        swap(temp_corners[0], temp_corners[1]);
    if (v2.cross(v3) < 0)
        swap(temp_corners[2], temp_corners[3]);

    rune_fan->getImageCache().setCorners(temp_corners);
    return true;
}

Contour_cptr RuneFanInactive::getEndArrowContour(FeatureNode_ptr &inactive_fan, const vector<FeatureNode_cptr> &inactive_targets)
{
    // 0. 判空
    if (inactive_targets.empty())
        return nullptr;
    if (inactive_fan == nullptr)
        return nullptr;
    auto fan = RuneFanInactive::cast(inactive_fan);
    if (fan->getArrowContours().size() < 2) // 轮廓数量为1时，不进行过滤
        return nullptr;

    FeatureNode_cptr ref_target = nullptr;
    vector<Contour_cptr> arrow_contours = fan->getArrowContours();
    if (inactive_targets.size() == 1)
    {
        ref_target = inactive_targets.front();
    }
    else
    {
        unordered_map<Contour_cptr, double> contour_weights{};
        double area_sum = 0;
        for (auto &contour : arrow_contours)
            area_sum += contour->area();

        if (area_sum == 0)
            return nullptr;

        for (auto &contour : arrow_contours)
            contour_weights[contour] = contour->area() / area_sum;
        
        Point2f all_contours_center{0, 0};
        for (auto &contour : arrow_contours)
            all_contours_center += static_cast<Point2f>(contour->center()) * contour_weights[contour];

        auto near_target = *min_element(inactive_targets.begin(), inactive_targets.end(), [&](const FeatureNode_cptr &a, const FeatureNode_cptr &b)
                                        { return norm(a->getImageCache().getCenter() - all_contours_center) < norm(b->getImageCache().getCenter() - all_contours_center); });
        ref_target = near_target;
    }

    auto far_contour = *max_element(arrow_contours.begin(), arrow_contours.end(), [&](const Contour_cptr &a, const Contour_cptr &b)
                                    { return norm(static_cast<Point2f>(a->center()) - ref_target->getImageCache().getCenter()) < norm(static_cast<Point2f>(b->center()) - ref_target->getImageCache().getCenter()); });

    vector<Contour_cptr> rest_contours = arrow_contours;
    rest_contours.erase(remove_if(rest_contours.begin(), rest_contours.end(), [&](const Contour_cptr &contour)
                                  { return contour == far_contour; }),
                        rest_contours.end());
    auto try_make_fan = RuneFanInactive::make_feature(rest_contours);
    if (try_make_fan == nullptr)
    {
        return nullptr;
    }
    fan = try_make_fan;
    inactive_fan = static_cast<FeatureNode_ptr>(fan);
    return far_contour;
}


auto RuneFanInactive::getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>
{
    vector<Point2f> points_2d{};
    vector<Point3f> points_3d{};
    vector<float> weights{};

    const auto& corners = getImageCache().getCorners();
    points_2d.push_back(corners[0]);
    points_2d.push_back(corners[1]);
    points_3d.push_back(rune_fan_param.INACTIVE_3D[0]);
    points_3d.push_back(rune_fan_param.INACTIVE_3D[1]);
    
    if (points_2d.size() != points_3d.size())
    {
        VC_THROW_ERROR("The size of points_2d and points_3d must be equal");
    }
    weights.resize(points_2d.size(), 1.0);

    return make_tuple(points_2d, points_3d, weights);
}

void RuneFanInactive::drawFeature(cv::Mat &image, const FeatureNode::DrawConfig_cptr &config) const
{
    // 绘制角点
    do
    {
        const auto& image_info = getImageCache();
        if(!image_info.isSetCorners())
            break;
        const auto& corners = image_info.getCorners();
        for(int i = 0 ; i < static_cast<int>(corners.size()); i++)
        {
            auto color = rune_fan_draw_param.inactive.color;
            auto thickness = rune_fan_draw_param.inactive.thickness;
            line(image, corners[i], corners[(i + 1) % corners.size()], color, thickness, LINE_AA);
            auto point_radius = rune_fan_draw_param.inactive.point_radius;
            circle(image, corners[i], point_radius, color, thickness, LINE_AA);
        }
        for(int i = 0 ; i < static_cast<int>(corners.size()); i++)
        {
            auto font_scale = rune_fan_draw_param.inactive.font_scale;
            auto font_thickness = rune_fan_draw_param.inactive.font_thickness;
            auto font_color = rune_fan_draw_param.inactive.font_color;
            putText(image, to_string(i), corners[i], FONT_HERSHEY_SIMPLEX, font_scale, font_color, font_thickness, LINE_AA);
        }

    } while (0);

    // 绘制方向
    do
    {
        if(!getImageCache().isSetDirection())
            break;
        auto arrow_thickness = rune_fan_draw_param.inactive.arrow_thickness;
        auto arrow_length = rune_fan_draw_param.inactive.arrow_length;
        auto arrow_color = rune_fan_draw_param.inactive.arrow_color;
        const auto& image_info = getImageCache();
        if(!image_info.isSetCenter())
            break;
        auto center = image_info.getCenter();
        auto direction = getImageCache().getDirection();
        if (direction == Point2f(0, 0))
            break;
        cv::arrowedLine(image, center, center + direction * arrow_length, arrow_color, arrow_thickness, LINE_AA, 0, 0.1);
    }while(0);
    
}