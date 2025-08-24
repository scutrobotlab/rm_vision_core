#include <numeric>
#include "vc/feature/rune_fan_active.h"
#include "vc/feature/rune_fan_param.h"
#include "vc/feature/rune_fan_hump_param.h"
#include "vc/core/debug_tools.h"

using namespace std;
using namespace cv;

#define RUNE_FAN_DEBUG 1

inline bool isHierarchyActiveFan(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t idx)
{
    return hierarchy[idx][3] == -1;
}

void RuneFanActive::find(vector<FeatureNode_ptr> &fans,
                         const vector<Contour_cptr> &contours,
                         const vector<Vec4i> &hierarchy,
                         const unordered_set<size_t> &mask,
                         unordered_map<FeatureNode_cptr, unordered_set<size_t>> &used_contour_idxs)
{
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (mask.count(i) || contours[i]->points().size() < 6)
            continue;
        if (isHierarchyActiveFan(contours, hierarchy, i))
        {
            auto p_fan = RuneFanActive::make_feature(contours[i]);
            if (p_fan)
            {
                fans.push_back(p_fan);
                used_contour_idxs[p_fan] = {i};
            }
        }
    }
}

shared_ptr<RuneFanActive> RuneFanActive::make_feature(const Contour_cptr &contour)
{
    auto rotated_rect = contour->minAreaRect();
    auto area = contour->area(), perimeter = contour->perimeter();
    double width = max(rotated_rect.size.width, rotated_rect.size.height);
    double height = min(rotated_rect.size.width, rotated_rect.size.height);
    double side_ratio = width / height;
    if (side_ratio > rune_fan_param.ACTIVE_MAX_SIDE_RATIO)
        return nullptr;
    double rect_area = rotated_rect.size.area(), area_ratio = area / rect_area;
    if (area_ratio > rune_fan_param.ACTIVE_MAX_AREA_RATIO || area_ratio < rune_fan_param.ACTIVE_MIN_AREA_RATIO)
        return nullptr;
    double area_perimeter_ratio = area / (perimeter * perimeter);
    if (area_perimeter_ratio > rune_fan_param.ACTIVE_MAX_AREA_PERIMETER_RATIO || area_perimeter_ratio < rune_fan_param.ACTIVE_MIN_AREA_PERIMETER_RATIO)
        return nullptr;
    if (area < rune_fan_param.ACTIVE_MIN_AREA || area > rune_fan_param.ACTIVE_MAX_AREA)
        return nullptr;

    vector<Point2f> top_hump_corners, bottom_center_hump_corners, side_hump_corners, bottom_side_hump_corners;
    if (!getActiveFunCorners(contour, top_hump_corners, bottom_center_hump_corners, side_hump_corners, bottom_side_hump_corners))
        return nullptr;
    return make_shared<RuneFanActive>(contour, rotated_rect, top_hump_corners, bottom_center_hump_corners, side_hump_corners, bottom_side_hump_corners);
}

RuneFanActive_ptr RuneFanActive::make_feature(const vector<Point2d> &top_corners,
                                              const vector<Point2d> &bottom_center_corners,
                                              const vector<Point2d> &side_corners,
                                              const vector<Point2d> &bottom_side_corners)
{
    auto convert_point = [](const vector<Point2d> &src, vector<Point2f> &dst)
    {
        dst.resize(src.size());
        for (size_t i = 0; i < src.size(); i++)
            dst[i] = Point2f(static_cast<float>(src[i].x), static_cast<float>(src[i].y));
    };
    vector<Point2f> top_corners_f, bottom_center_corners_f, side_corners_f, bottom_side_corners_f;
    convert_point(top_corners, top_corners_f);
    convert_point(bottom_center_corners, bottom_center_corners_f);
    convert_point(side_corners, side_corners_f);
    convert_point(bottom_side_corners, bottom_side_corners_f);

    vector<Point2f> points;
    points.insert(points.end(), top_corners_f.begin(), top_corners_f.end());
    points.insert(points.end(), bottom_center_corners_f.begin(), bottom_center_corners_f.end());
    points.insert(points.end(), side_corners_f.begin(), side_corners_f.end());
    points.insert(points.end(), bottom_side_corners_f.begin(), bottom_side_corners_f.end());
    vector<Point2f> hull;
    convexHull(points, hull);
    auto contour = ContourWrapper<int>::make_contour(vector<Point>(hull.begin(), hull.end()));
    return make_shared<RuneFanActive>(contour, contour->minAreaRect(), top_corners_f, bottom_center_corners_f, side_corners_f, bottom_side_corners_f);
}

Mat RuneFanActive::getAngles(const vector<Point> &contour_plus)
{
    Mat contours_mat(2, contour_plus.size(), CV_32F);
    auto *mat_p_x = contours_mat.ptr<float>(0), *mat_p_y = contours_mat.ptr<float>(1);
    for (const auto &p : contour_plus)
    {
        *mat_p_x++ = p.x;
        *mat_p_y++ = p.y;
    }

    // 获取方向数组，并进行一定预处理
    Mat directions_mat(contours_mat.size(), CV_32F), direction_kernel = (Mat_<float>(1, 2) << -1, 1);
    filter2D(contours_mat, directions_mat, -1, direction_kernel, Point(0, 0), 0, BORDER_DEFAULT);
    int filter_len = max(17, min(static_cast<int>(rune_fan_hump_param.TOP_HUMP_FILTER_LEN_RATIO * contour_plus.size()) | 1, 101));
    Mat kernel = getGaussianKernel(filter_len, rune_fan_hump_param.TOP_HUMP_FILTER_SIGMA, CV_32F).t();
    normalize(kernel, kernel, 1, 0, NORM_L1);
    filter2D(directions_mat, directions_mat, -1, kernel, Point(-1, -1), 0, BORDER_DEFAULT);

    // 角度数组，均使用OpenCV的Mat存储，便于计算
    Mat angles(1, directions_mat.cols, CV_32FC1);
    auto p_x = directions_mat.ptr<float>(0), p_y = directions_mat.ptr<float>(1), angle_p = angles.ptr<float>(0);
    int n = 0;
    float last_angle = 0;
    for (int i = 0; i < directions_mat.cols; i++)
    {
        float angle = rad2deg(atan2(*p_y++, *p_x++));
        if (angle - last_angle < -180)
            n++;
        else if (angle - last_angle > 180)
            n--;
        last_angle = angle;
        *angle_p++ = angle + n * 360;
    }
    return angles;
}

// 梯度计算函数
Mat RuneFanActive::getGradient(const Mat &angles_mat)
{
    Mat gradient_mat(1, angles_mat.cols, CV_32F);
    Mat kernel = (Mat_<float>(1, 3) << -1, 0, 1);
    filter2D(angles_mat, gradient_mat, -1, kernel, Point(-1, -1), 0, BORDER_DEFAULT);
    return gradient_mat;
}
// 获取所有的线段
inline bool getAllLine(const vector<Point> &contour_plus, const Mat &angles_mat, const Mat &gradient, std::vector<Line> &lines)
{
    if (angles_mat.cols != gradient.cols)
        VC_THROW_ERROR("The cols of angles and gradient must be equal");
    vector<tuple<size_t, float, size_t, float>> temp_lines;
    const float *p_angles = angles_mat.ptr<float>(0);
    const float *p_gradient = gradient.ptr<float>(0);
    bool is_in_line = false;

    for (size_t i = 0; i < angles_mat.cols; i++, p_angles++, p_gradient++)
    {
        float angle = *p_angles, grad = *p_gradient;
        if (abs(grad) <= 3)
        {
            if (!is_in_line)
            {
                is_in_line = true;
                temp_lines.emplace_back(i, angle, i, angle);
            }
            else
            {
                auto &[s_idx, s_angle, e_idx, e_angle] = temp_lines.back();
                e_idx = i;
                e_angle = angle;
            }
        }
        else if (is_in_line)
        {
            is_in_line = false;
        }
    }

    for (size_t i = 1; i < temp_lines.size(); i++)
    {
        auto &[prev_s, prev_sa, prev_e, prev_ea] = temp_lines[i - 1];
        auto &[next_s, next_sa, next_e, next_ea] = temp_lines[i];
        if (next_s - prev_e < 20 && abs(next_ea - prev_sa) < 5)
        {
            prev_e = next_e;
            prev_ea = next_ea;
            temp_lines.erase(temp_lines.begin() + i);
            i--;
        }
    }

    lines.clear();
    for (auto &[s_idx, s_angle, e_idx, e_angle] : temp_lines)
    {
        float angle = accumulate(angles_mat.ptr<float>(0) + s_idx, angles_mat.ptr<float>(0) + e_idx + 1, 0.0f) / (e_idx - s_idx + 1);
        Point2f center = static_cast<Point2f>(accumulate(contour_plus.begin() + s_idx, contour_plus.begin() + e_idx + 1, Point(0, 0))) / float(e_idx - s_idx + 1);
        lines.emplace_back(s_idx, e_idx, angle, center);
    }
    return true;
}

// 正反线段匹配，将方向相反的两个邻近线段进行一一配对
inline bool matchLine(const vector<Point> contour_plus, const Mat &angles_mat, std::vector<Line> &lines, std::vector<tuple<Line, Line>> &matched_lines)
{
    RotatedRect rect = minAreaRect(contour_plus);
    float max_vertical_distance = max(rect.size.height, rect.size.width) / 2.0f;
    float max_angle_delta = 20.0f;

    auto getLineVerticalDistance = [](const Line &l1, const Line &l2) -> float
    {
        float len1 = l1.end_idx - l1.start_idx + 1, len2 = l2.end_idx - l2.start_idx + 1;
        float ave_angle = (l1.angle * len1 + l2.angle * len2) / (len1 + len2);
        return getDist(l1.center, l2.center) * abs(cos(deg2rad(abs(ave_angle - l1.angle))));
    };

    vector<tuple<size_t, size_t>> matched_idxs;
    for (size_t i = 0; i < lines.size(); i++)
    {
        float last_vertical_distance = 1e6;
        size_t matched_j = -1;
        for (size_t j = i + 1; j < lines.size(); j++)
        {
            if (abs(lines[i].angle - lines[j].angle) > 360)
                break;
            if (abs(lines[i].end_idx - lines[j].start_idx) > contour_plus.size() / 3.0)
                break;
            float delta_angle = abs(lines[j].angle - lines[i].angle);
            if (abs(delta_angle - 180) > max_angle_delta)
                continue;
            float v_dist = getLineVerticalDistance(lines[i], lines[j]);
            if (v_dist > max_vertical_distance || v_dist > last_vertical_distance)
                continue;
            last_vertical_distance = v_dist;
            matched_j = j;
        }
        if (matched_j != size_t(-1))
            matched_idxs.emplace_back(i, matched_j);
    }

    for (size_t i = 0; i < matched_idxs.size(); i++)
    {
        auto &[up1, down1] = matched_idxs[i];
        for (size_t j = i + 1; j < matched_idxs.size(); j++)
        {
            auto &[up2, down2] = matched_idxs[j];
            if (down1 == down2)
            {
                Line &l1 = lines[up1], &l2 = lines[up2];
                matched_idxs.erase(matched_idxs.begin() + j);
                if (getLineVerticalDistance(l1, l2) < 5)
                {
                    l1.angle = (l1.angle * (l1.end_idx - l1.start_idx + 1) + l2.angle * (l2.end_idx - l2.start_idx + 1)) /
                               (l1.end_idx - l1.start_idx + 1 + l2.end_idx - l2.start_idx + 1);
                    l1.start_idx = min(l1.start_idx, l2.start_idx);
                    l1.end_idx = max(l1.end_idx, l2.end_idx);
                }
            }
        }
    }

    matched_idxs.erase(remove_if(matched_idxs.begin(), matched_idxs.end(), [&](const tuple<size_t, size_t> &idx)
                                 {
        const Line &up = lines[get<0>(idx)], &down = lines[get<1>(idx)];
        Point2f dir = Point2f(cos(deg2rad(up.angle)), sin(deg2rad(up.angle)));
        return dir.cross(down.center - up.center) > 0; }),
                       matched_idxs.end());

    matched_lines.clear();
    for (auto &[up_idx, down_idx] : matched_idxs)
        matched_lines.emplace_back(lines[up_idx], lines[down_idx]);
    return true;
}

// 线段角度矫正，利用正反线段的相关关系进行矫正
inline void correctLineAngle(std::vector<std::tuple<Line, Line>> &line_pairs)
{
    for (auto &[up_line, down_line] : line_pairs)
    {
        auto normalizeAngle = [](float &angle)
        {
            while (angle > 180)
                angle -= 360;
            while (angle < -180)
                angle += 360;
        };
        normalizeAngle(up_line.angle);
        normalizeAngle(down_line.angle);

        float ave_angle = (up_line.angle + down_line.angle) / 2.0f;
        up_line.angle = up_line.angle > ave_angle ? ave_angle + 90 : ave_angle - 90;
        down_line.angle = down_line.angle > ave_angle ? ave_angle + 90 : ave_angle - 90;

        normalizeAngle(up_line.angle);
        normalizeAngle(down_line.angle);
    }
}

// 重合线段对合并
inline bool mergeLinePairs(const vector<Point> &contour_plus, std::vector<std::tuple<Line, Line>> &line_pairs)
{
    static const float max_distance = 20;
    static const float max_angle_delta = 10;
    static const float max_len_ratio = 3.0;

    auto getDeltaAngle = [](const Line &l1, const Line &l2) -> float
    {
        float delta_angle = abs(l1.angle - l2.angle);
        if (delta_angle > 360)
            delta_angle -= 360;
        if (delta_angle > 180)
            delta_angle = 360 - delta_angle;
        return delta_angle;
    };

    // up: 上半部分线段，down: 下半部分线段
    for (int i = 0; i < static_cast<int>(line_pairs.size()); i++)
    {
        auto &[up1, down1] = line_pairs[i];
        Point2f center1 = (up1.center + down1.center) / 2.0f;

        for (int j = i + 1; j < static_cast<int>(line_pairs.size()); j++)
        {
            auto &[up2, down2] = line_pairs[j];
            Point2f center2 = (up2.center + down2.center) / 2.0f;
            if (getDist(center1, center2) > max_distance)
                continue;

            float up_delta = getDeltaAngle(up1, up2);
            float down_delta = getDeltaAngle(down1, down2);
            if (up_delta > max_angle_delta || down_delta > max_angle_delta)
                continue;

            float up_len1 = up1.end_idx - up1.start_idx + 1;
            float up_len2 = up2.end_idx - up2.start_idx + 1;
            float down_len1 = down1.end_idx - down1.start_idx + 1;
            float down_len2 = down2.end_idx - down2.start_idx + 1;

            if (max(up_len1, up_len2) / min(up_len1, up_len2) < max_len_ratio)
            {
                Point2f dir = (Point2f(cos(deg2rad(up1.angle)), sin(deg2rad(up1.angle))) * up_len1 +
                               Point2f(cos(deg2rad(up2.angle)), sin(deg2rad(up2.angle))) * up_len2) /
                              (up_len1 + up_len2);
                up1.angle = rad2deg(atan2(dir.y, dir.x));
            }
            else
            {
                up1.angle = up_len1 > up_len2 ? up1.angle : up2.angle;
            }

            if (max(down_len1, down_len2) / min(down_len1, down_len2) < max_len_ratio)
            {
                Point2f dir = (Point2f(cos(deg2rad(down1.angle)), sin(deg2rad(down1.angle))) * down_len1 +
                               Point2f(cos(deg2rad(down2.angle)), sin(deg2rad(down2.angle))) * down_len2) /
                              (down_len1 + down_len2);
                down1.angle = rad2deg(atan2(dir.y, dir.x));
            }
            else
            {
                down1.angle = down_len1 > down_len2 ? down1.angle : down2.angle;
            }

            line_pairs.erase(line_pairs.begin() + j);
            j--;
        }
    }
    return true;
}

// 获取PnP点
auto RuneFanActive::getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>
{
    vector<Point2f> points_2d;
    vector<Point3f> points_3d;
    vector<float> weights;

    if (isSetTopHumpCorners())
    {
        auto &c = getTopHumpCorners();
        points_2d.insert(points_2d.end(), c.begin(), c.end());
        points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_TOP_3D.begin(), rune_fan_param.ACTIVE_TOP_3D.end());
    }
    if (isSetBottomCenterHumpCorners())
    {
        auto &c = getBottomCenterHumpCorners();
        points_2d.insert(points_2d.end(), c.begin(), c.end());
        points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_BOTTOM_CENTER_3D.begin(), rune_fan_param.ACTIVE_BOTTOM_CENTER_3D.end());
    }
    if (isSetSideHumpCorners())
    {
        auto &c = getSideHumpCorners();
        points_2d.insert(points_2d.end(), c.begin(), c.end());
        points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_SIDE_3D.begin(), rune_fan_param.ACTIVE_SIDE_3D.end());
    }
    if (isSetBottomSideHumpCorners())
    {
        auto &c = getBottomSideHumpCorners();
        points_2d.insert(points_2d.end(), c.begin(), c.end());
        points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_BOTTOM_SIDE_3D.begin(), rune_fan_param.ACTIVE_BOTTOM_SIDE_3D.end());
    }

    if (points_2d.size() != points_3d.size())
        VC_THROW_ERROR("The size of points_2d and points_3d must be equal");
    weights.resize(points_2d.size(), 1.0f);

    return {points_2d, points_3d, weights};
}

// 占位线段延长
inline bool extendLines(const vector<Point> &contour_plus, const std::vector<Line> &lines, std::vector<tuple<Line, Line>> &matched_idxs)
{
    return true;
}

// 获取线段对主流程
bool RuneFanActive::getLinePairs(const vector<Point> &contour_plus, const Mat &angles_mat, const Mat &gradient_mat, std::vector<std::tuple<Line, Line>> &line_pairs)
{
    vector<Line> lines;
    getAllLine(contour_plus, angles_mat, gradient_mat, lines);

    vector<tuple<Line, Line>> matched_lines;
    matchLine(contour_plus, angles_mat, lines, matched_lines);
    mergeLinePairs(contour_plus, matched_lines);
    correctLineAngle(matched_lines);
    extendLines(contour_plus, lines, matched_lines);

    line_pairs = matched_lines;
    return true;
}

bool RuneFanActive::getActiveFunCorners(const Contour_cptr &contour,
                                        std::vector<cv::Point2f> &top_hump_corners,
                                        std::vector<cv::Point2f> &bottom_center_hump_corners,
                                        std::vector<cv::Point2f> &side_hump_corners,
                                        std::vector<cv::Point2f> &bottom_side_hump_corners)
{
    Point2f center = contour->fittedEllipse().center;
    const auto &raw = contour->points();
    vector<Point> contour_plus(raw.begin(), raw.end());
    contour_plus.insert(contour_plus.end(), raw.begin(), raw.begin() + raw.size() / 3);
    Mat angles_mat = getAngles(contour_plus), gradient_mat = getGradient(angles_mat);

    vector<tuple<Line, Line>> line_pairs;
    getLinePairs(contour_plus, angles_mat, gradient_mat, line_pairs);
    if (line_pairs.size() < 4)
        return false;

    auto top_humps = TopHump::getTopHumps(contour_plus, center, line_pairs);
    if (top_humps.empty())
        return false;
    auto bottom_center_humps = BottomCenterHump::getBottomCenterHump(contour_plus, center, top_humps);
    if (bottom_center_humps.empty())
        return false;
    auto side_humps = SideHump::getSideHumps(contour_plus, center, top_humps, bottom_center_humps, line_pairs);

    top_hump_corners = top_humps.empty() ? vector<Point2f>{} : vector<Point2f>{top_humps[0].getVertex(), top_humps[1].getVertex(), top_humps[2].getVertex()};
    bottom_center_hump_corners = bottom_center_humps.empty() ? vector<Point2f>{} : vector<Point2f>{bottom_center_humps[0].getVertex()};
    side_hump_corners = side_humps.empty() ? vector<Point2f>{} : vector<Point2f>{side_humps[0].getVertex(), side_humps[1].getVertex()};
    bottom_side_hump_corners.clear();
    return true;
}

RuneFanActive::RuneFanActive(const Contour_cptr &contour,
                             const cv::RotatedRect &rotated_rect,
                             const std::vector<cv::Point2f> &top_hump_corners,
                             const std::vector<cv::Point2f> &bottom_center_hump_corners,
                             const std::vector<cv::Point2f> &side_hump_corners,
                             const std::vector<cv::Point2f> &bottom_side_hump_corners)
{

    if (!top_hump_corners.empty())
        setTopHumpCorners(top_hump_corners);
    if (!bottom_center_hump_corners.empty())
        setBottomCenterHumpCorners(bottom_center_hump_corners);
    if (!side_hump_corners.empty())
        setSideHumpCorners(side_hump_corners);
    if (!bottom_side_hump_corners.empty())
        setBottomSideHumpCorners(bottom_side_hump_corners);

    if (!isSetTopHumpCorners() || !isSetBottomCenterHumpCorners())
        VC_THROW_ERROR("top_hump_corners or bottom_center_hump_corners empty");

    // 根据角点的获取情况分配角点
    vector<Point2f> corners;
    if (isSetTopHumpCorners() && isSetBottomCenterHumpCorners())
        corners = {top_hump_corners[0], top_hump_corners[1], top_hump_corners[2], bottom_center_hump_corners[0]};
    if (isSetTopHumpCorners() && isSetBottomCenterHumpCorners() && !isSetSideHumpCorners() && isSetBottomSideHumpCorners())
        corners = {top_hump_corners[0], top_hump_corners[1], top_hump_corners[2], bottom_side_hump_corners[1], bottom_center_hump_corners[0], bottom_side_hump_corners[0]};
    if (isSetTopHumpCorners() && isSetBottomCenterHumpCorners() && isSetSideHumpCorners() && !isSetBottomSideHumpCorners())
        corners = {top_hump_corners[0], top_hump_corners[1], top_hump_corners[2], side_hump_corners[1], bottom_center_hump_corners[0], side_hump_corners[0]};
    if (isSetTopHumpCorners() && isSetBottomCenterHumpCorners() && isSetSideHumpCorners() && isSetBottomSideHumpCorners())
        corners = {top_hump_corners[0], top_hump_corners[1], top_hump_corners[2], side_hump_corners[1], bottom_side_hump_corners[1], bottom_center_hump_corners[0], bottom_side_hump_corners[0], side_hump_corners[0]};

    float width = getDist(top_hump_corners[0], top_hump_corners[2]);
    float height = getDist(top_hump_corners[1], bottom_center_hump_corners[0]);
    Point2f center = (top_hump_corners[1] + bottom_center_hump_corners[0]) / 2;
    auto direction = getUnitVector(bottom_center_hump_corners[0] - top_hump_corners[1]);
    setRotatedRect(rotated_rect);
    auto &img_info = this->getImageCache();
    img_info.setContours(vector<Contour_cptr>{contour});
    img_info.setCorners(corners);
    img_info.setWidth(width);
    img_info.setHeight(height);
    img_info.setCenter(center);
    img_info.setDirection(direction);
    setActiveFlag(true);
}

void RuneFanActive::drawFeature(cv::Mat &image, const FeatureNode::DrawConfig_cptr &config) const
{
    const auto &img_info = getImageCache();
    if (img_info.isSetCorners())
    {
        const auto &corners = img_info.getCorners();
        for (int i = 0; i < static_cast<int>(corners.size()); i++)
        {
            line(image, corners[i], corners[(i + 1) % corners.size()],
                 rune_fan_draw_param.active.color, rune_fan_draw_param.active.thickness, LINE_AA);
            circle(image, corners[i], rune_fan_draw_param.active.point_radius,
                   rune_fan_draw_param.active.color, rune_fan_draw_param.active.thickness, LINE_AA);
            putText(image, to_string(i), corners[i], FONT_HERSHEY_SIMPLEX,
                    rune_fan_draw_param.active.font_scale, rune_fan_draw_param.active.font_color,
                    rune_fan_draw_param.active.font_thickness, LINE_AA);
        }
    }
    if (img_info.isSetCenter() && img_info.isSetDirection())
    {
        auto center = img_info.getCenter();
        auto dir = img_info.getDirection();
        if (dir != Point2f(0, 0))
            cv::arrowedLine(image, center, center + dir * rune_fan_draw_param.active.arrow_length,
                            rune_fan_draw_param.active.arrow_color, rune_fan_draw_param.active.arrow_thickness, LINE_AA, 0, 0.1);
    }
}
