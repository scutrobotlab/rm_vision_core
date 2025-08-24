#include <numeric>

#include "vc/detector/rune_detector.h"
#include "vc/detector/rune_detector_param.h"
#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_target.h"
#include "vc/feature/rune_center.h"
#include "vc/feature/rune_fan_param.h"

using namespace std;
using namespace cv;

#define rune_detector_debug

struct RuneDetectorFilterParam
{
    float MIN_RADIUS_RATIO = 1.f;   // 特征间距与神符中心高度的最小比值
    float MAX_RADIUS_RATIO = 15.f;  // 特征间距与神符中心高度的最大比值
    float BEST_RADIUS_RATIO = 11.f; // 特征间距与神符中心高度的最优比值
    float MAX_MID_LINE_RATIO = 2.f; // 神符中心离扇叶间中垂线距离与神符中心高度的最大比值
};

inline float getRotatedRectArea(const FeatureNode_cptr feature)
{
    if (!feature)
        return 0.f;
    return feature->getImageCache().getContours().front()->minAreaRect().size.area();
}

inline auto getRotatedRect(const FeatureNode_cptr feature) -> RotatedRect
{
    if (!feature)
        return RotatedRect();
    return feature->getImageCache().getContours().front()->minAreaRect();
}

inline float getArea(const FeatureNode_cptr feature)
{
    if (!feature)
        return 0.f;
    return feature->getImageCache().getContours().front()->area();
}

inline Point2f getCenter(const FeatureNode_cptr feature)
{
    if (!feature)
        return Point2f(0.f, 0.f);
    return feature->getImageCache().getCenter();
}

void RuneDetector::binary(const cv::Mat &src, cv::Mat &bin, PixChannel target_color, uint8_t threshold)
{
    if(src.type() != CV_8UC3)
    {
        VC_THROW_ERROR("The image type of \"src\" is incorrect");
    }
    if(target_color != PixChannel::RED && target_color != PixChannel::BLUE)
    {
        VC_THROW_ERROR("The value of \"target_color\" is incorrect");
    }
    auto ch1 = target_color == PixChannel::RED ? PixChannel::RED : PixChannel::BLUE;
    auto ch2 = target_color == PixChannel::RED ? PixChannel::BLUE : PixChannel::RED;
    bin = Mat::zeros(Size(src.cols, src.rows), CV_8UC1);
    parallel_for_(Range(0, src.rows),
                    [&](const Range &range)
                    {
                        const uchar *data_src = nullptr;
                        uchar *data_bin = nullptr;
                        for (int row = range.start; row < range.end; ++row)
                        {
                            data_src = src.ptr<uchar>(row);
                            data_bin = bin.ptr<uchar>(row);
                            for (int col = 0; col < src.cols; ++col)
                                if (data_src[3 * col + ch1] - data_src[3 * col + ch2] > threshold)
                                    data_bin[col] = 255;
                        }
                    });
}

bool RuneDetector::filterInactiveTarget(std::vector<FeatureNode_ptr> &inactive_targets)
{
    if (inactive_targets.size() > 1)
    {
        // 选择面积最大的未激活靶心
        auto max_target = *max_element(inactive_targets.begin(), inactive_targets.end(), [](const FeatureNode_ptr &t1, const FeatureNode_ptr &t2)
                                       { return t1->getImageCache().getContours().front()->minAreaRect().size.area() < t2->getImageCache().getContours().front()->minAreaRect().size.area(); });
        inactive_targets.clear();
        inactive_targets.push_back(max_target);
    }

    return true;
}

bool RuneDetector::filterActiveTarget(std::vector<FeatureNode_ptr> &active_targets, std::vector<FeatureNode_ptr> &inactive_targets)
{
    if (active_targets.size() > 4)
    {
        Point2f ave_center{};
        for (auto &target : active_targets)
            ave_center += target->getImageCache().getCenter();
        ave_center /= static_cast<float>(active_targets.size());

        // 选择距离中心最近的4个激活靶心
        sort(active_targets.begin(), active_targets.end(), [&ave_center](const FeatureNode_ptr &t1, const FeatureNode_ptr &t2)
             { return getDist(t1->getImageCache().getCenter(), ave_center) < getDist(t2->getImageCache().getCenter(), ave_center); });
        active_targets.erase(active_targets.begin() + 4, active_targets.end());
    }
    return true;
}

inline bool filterInactiveFanImpl(vector<FeatureNode_ptr> &inactive_fans,
                                  const vector<FeatureNode_ptr> &inactive_targets,
                                  const vector<FeatureNode_ptr> &active_targets,
                                  const vector<FeatureNode_ptr> &active_fans)
{
    if (inactive_fans.empty())
        return false;
    if (inactive_targets.empty() && active_targets.empty() && active_fans.empty())
    {
        // 当没有参考特征时，采用面积最大的未激活扇叶
        auto max_fan = *max_element(inactive_fans.begin(), inactive_fans.end(), [](const FeatureNode_ptr &f1, const FeatureNode_ptr &f2)
                                    { return getRotatedRectArea(f1) < getRotatedRectArea(f2); });
        inactive_fans.clear();
        inactive_fans.push_back(max_fan);
        return false;
    }

    // 初始化灯臂特征的正确率
    unordered_map<FeatureNode_ptr, double> fan_accuracy_map;
    for (auto &inactive_fan : inactive_fans)
        fan_accuracy_map[inactive_fan] = 1.0;

    // 利用神符靶心筛选
    if (inactive_targets.size() + active_targets.size() >= 3)
    {
        vector<Point> target_centers{};
        for (auto &p_target : inactive_targets)
            target_centers.push_back(getCenter(p_target));
        for (auto &p_target : active_targets)
            target_centers.push_back(getCenter(p_target));

        // 拟合圆
        Point2f center;
        float radius;
        minEnclosingCircle(target_centers, center, radius);
        double target_accuracy = 1.0;
        for (auto &inactive_target : inactive_targets)
            target_accuracy *= rune_detector_param.INACTIVE_TARGET_ACCURACY;
        for (auto &active_target : active_targets)
            target_accuracy *= rune_detector_param.ACTIVE_TARGET_ACCURACY;
        for (auto &inactive_fan : inactive_fans)
        {
            double distance = getDist(inactive_fan->getImageCache().getCenter(), center);
            if (distance > radius)
                fan_accuracy_map[inactive_fan] *= (1 - target_accuracy);
        }
    }

    // 利用未激活神符靶心筛选   （要求未激活神符靶心的准确度要足够高）
    if (!inactive_targets.empty())
    {
        auto best_target = inactive_targets[0];
        for (auto &fan : inactive_fans)
        {
            Point2f direction = fan->getImageCache().getDirection(); // 扇叶的方向

            Point2f fan_to_target = getCenter(best_target) - getCenter(fan);
            float delta_angle = getVectorMinAngle(direction, fan_to_target, DEG);

            if (delta_angle > 90)
                delta_angle = 180 - delta_angle;
            if (delta_angle > 10)
            {

                // fan_accuracy_map[fan] *= 0;
                fan_accuracy_map[fan] *= (1 - rune_detector_param.INACTIVE_TARGET_ACCURACY);
            }
        }
    }

    // 利用已激活扇叶的方向筛选
    if (active_fans.size() >= 2)
    {
        vector<Point2f> temp_centers{};
        for (size_t i = 0; i < active_fans.size(); i++)
        {
            for (size_t j = i + 1; j < active_fans.size(); j++)
            {
                const auto &fan1 = active_fans[i];
                const auto &fan2 = active_fans[j];
                Vec4f line1{getCenter(fan1).x, getCenter(fan1).y, getCenter(fan1).x + fan1->getImageCache().getDirection().x, getCenter(fan1).y + fan1->getImageCache().getDirection().y};
                Vec4f line2{getCenter(fan2).x, getCenter(fan2).y, getCenter(fan2).x + fan2->getImageCache().getDirection().x, getCenter(fan2).y + fan2->getImageCache().getDirection().y};
                Point2f cross_point = getCrossPoint(line1, line2);
                temp_centers.emplace_back(cross_point);
            }
        }
        Point2f ave_center = std::accumulate(temp_centers.begin(), temp_centers.end(), Point2f(0, 0)) / static_cast<double>(temp_centers.size()); // 平均中心
        // 设置已激活扇叶的正确率
        double fan_accuracy = 1.0;
        for (auto &active_fan : active_fans)
            fan_accuracy *= rune_detector_param.ACTIVE_FAN_ACCURACY;
        // 更新所有灯臂的正确率
        for (auto &inactive_fan : inactive_fans)
        {
            Vec4f direction_line{inactive_fan->getImageCache().getDirection().x, inactive_fan->getImageCache().getDirection().y, getCenter(inactive_fan).x, getCenter(inactive_fan).y};
            if (getDist(direction_line, ave_center) > max(inactive_fan->getImageCache().getHeight(), inactive_fan->getImageCache().getWidth()))
                fan_accuracy_map[inactive_fan] *= (1 - fan_accuracy);
        }
    }

    // 删除正确率过低的灯臂
    for (auto it = inactive_fans.begin(); it != inactive_fans.end();)
    {
        if (fan_accuracy_map[*it] < rune_detector_param.MIN_INACTIVE_FAN_ACCURACY)
        {
            it = inactive_fans.erase(it);
        }
        else
            ++it;
    }

    if (inactive_fans.empty())
        return false;

    // 对重合的灯臂、保留面积最大的那个
    if (inactive_fans.size() >= 2)
    {
        for (size_t i = 0; i < inactive_fans.size(); i++)
        {
            for (size_t j = i + 1; j < inactive_fans.size(); j++)
            {
                const auto &fan1 = inactive_fans[i];
                const auto &fan2 = inactive_fans[j];
                const auto &points_1 = fan1->getImageCache().getContours().front()->points();
                const auto &points_2 = fan2->getImageCache().getContours().front()->points();
                if (pointPolygonTest(points_1, getCenter(fan2), false) > 0 || pointPolygonTest(points_2, getCenter(fan1), false) > 0)
                {
                    if (fan1->getImageCache().getContours().front()->area() > fan2->getImageCache().getContours().front()->area())
                        inactive_fans.erase(inactive_fans.begin() + j);
                    else
                        inactive_fans.erase(inactive_fans.begin() + i);
                }
            }
        }
    }

    // 计算扇叶到所有未激活靶心的平均距离
    unordered_map<FeatureNode_ptr, double> fan_distance;
    for (auto &inactive_fan : inactive_fans)
    {
        double distance = 0;
        for (auto &inactive_target : inactive_targets)
            distance += getDist(getCenter(inactive_fan), getCenter(inactive_target));
        distance /= static_cast<float>(inactive_targets.size());
        fan_distance[inactive_fan] = distance;
    }

    auto [min_distance_fan, distance] = *min_element(fan_distance.begin(), fan_distance.end(), [](const auto &lhs, const auto &rhs)
                                                     { return lhs.second < rhs.second; });

    auto &best_fan = min_distance_fan;

    // 单独判断剩下来的灯臂距离其它特征的是否过远。
    float max_dis = 0;
    for (auto &inactive_target : inactive_targets)
        max_dis = max(max_dis, getDist(getCenter(best_fan), getCenter(inactive_target)));
    for (auto &active_target : active_targets)
        max_dis = max(max_dis, getDist(getCenter(best_fan), getCenter(active_target)));
    for (auto &active_fan : active_fans)
        max_dis = max(max_dis, getDist(getCenter(best_fan), getCenter(active_fan)));

    if (max_dis > max(best_fan->getImageCache().getHeight(), best_fan->getImageCache().getWidth()) * rune_fan_param.INACTIVE_MAX_DISTANCE_RATIO)
    {
        inactive_fans.clear();
        return false;
    }
    inactive_fans.clear();
    inactive_fans.push_back(best_fan);

    return true;
}

bool RuneDetector::filterInactiveFan(vector<FeatureNode_ptr> &inactive_fans,
                                     const vector<FeatureNode_ptr> &inactive_targets,
                                     const vector<FeatureNode_ptr> &active_targets,
                                     const vector<FeatureNode_ptr> &active_fans)
{
    // 保存筛选前的扇叶
    unordered_set<FeatureNode_ptr> erased_fan{};
    erased_fan.insert(inactive_fans.begin(), inactive_fans.end());

    bool result = filterInactiveFanImpl(inactive_fans, inactive_targets, active_targets, active_fans);

    for (auto &used_fan : inactive_fans)
    {
        if (erased_fan.find(used_fan) == erased_fan.end())
            continue;
        erased_fan.erase(used_fan);
    }

    return result;
}

struct RuneDetectorFilterCenterParam
{
    //! 神符中心距离中垂线的距离与神符中心直径的距离比例
    float MAX_TARGET_MID_LINE_DISTANCE_RATIO = 5.f;
    //! 神符中心到靶心的距离与靶心直径的距离比例
    float MAX_TARGET_DISTANCE_RATIO = 3.f;
    float MIN_TARGET_DISTANCE_RATIO = 1.0f;

    //! 神符中心与扇叶所成直线与扇叶方向的的最大夹角
    float MAX_FAN_ANGLE = 30.f;
    //! 神符中心到扇叶所在直线的距离与神符中心直径的距离比例
    float MAX_FAN_LINE_DISTANCE_RATIO = 5.f;
    //! 神符中心到扇叶的距离与扇叶直径的距离比例
    float MAX_FAN_DISTANCE_RATIO = 2.f;

    YML_INIT(
        RuneDetectorFilterCenterParam,
        YML_ADD_PARAM(MAX_TARGET_MID_LINE_DISTANCE_RATIO);
        YML_ADD_PARAM(MAX_TARGET_DISTANCE_RATIO);
        YML_ADD_PARAM(MIN_TARGET_DISTANCE_RATIO);
        YML_ADD_PARAM(MAX_FAN_ANGLE);
        YML_ADD_PARAM(MAX_FAN_LINE_DISTANCE_RATIO);
        YML_ADD_PARAM(MAX_FAN_DISTANCE_RATIO););
};
inline RuneDetectorFilterCenterParam rune_detector_filter_center_param;

bool RuneDetector::filterCenter(std::vector<FeatureNode_ptr> &centers,
                                const std::vector<FeatureNode_ptr> &inactive_targets,
                                const std::vector<FeatureNode_ptr> &active_targets,
                                const std::vector<FeatureNode_ptr> &inactive_fans,
                                const std::vector<FeatureNode_ptr> &active_fans)
{
    if (centers.empty())
        return false;
    if (inactive_targets.empty() && active_targets.empty() && inactive_fans.empty() && active_fans.empty())
    {
        // 当没有参考特征时，采用面积最大的神符中心
        auto max_center = *max_element(centers.begin(), centers.end(), [](const FeatureNode_ptr &c1, const FeatureNode_ptr &c2)
                                       { return c1->getImageCache().getContours().front()->area() < c2->getImageCache().getContours().front()->area(); });
        return false;
    }
    if (centers.size() > 30)
    {
        sort(centers.begin(), centers.end(), [](const FeatureNode_ptr &c1, const FeatureNode_ptr &c2)
             { return getArea(c1) > getArea(c2); });
        centers.erase(centers.begin() + 30, centers.end());
    }

    // 设置各个神符中心的正确率
    unordered_map<FeatureNode_ptr, double> center_accuracy;
    for (auto &center : centers)
        center_accuracy[center] = 1; // 初始化为正确概率为 1.0

    // 特殊筛选（1）
    if (inactive_targets.size() == 1 &&
        inactive_fans.size() == 1)
    {
        auto target_rect = inactive_targets.front()->getImageCache().getContours().front()->minAreaRect();
        auto fan_rect = inactive_fans.front()->getImageCache().getContours().front()->minAreaRect();
        Point2f fan_center = fan_rect.center;
        Point2f target_center = target_rect.center;
        Point2f sym_target_center = fan_center + (fan_center - target_center);
        float radius = sqrt(target_rect.size.area() / CV_PI);
        constexpr float SCALE_FACTOR = 1.6f;
        radius *= SCALE_FACTOR;

        // 对不在圈内的神符中心进行筛选
        for (auto it = centers.begin(); it != centers.end();)
        {
            float distance = getDist(sym_target_center, getCenter(*it));
            if (distance > radius)
            {
                center_accuracy[*it] *= 0; 
                it = centers.erase(it);
            }
            else
            {
                ++it; 
            }
        }
    }

    // 通过未激活靶心筛选
    vector<RuneTarget_ptr> all_targets{};
    for (size_t i = 0; i < inactive_targets.size(); i++)
        all_targets.push_back(RuneTarget::cast(inactive_targets[i]));
    for (size_t i = 0; i < active_targets.size(); i++)
        all_targets.push_back(RuneTarget::cast(active_targets[i]));

    for (size_t i = 0; i < all_targets.size(); i++)
    {
        // ---------------------中垂线判断----------------------
        for (size_t j = i + 1; j < all_targets.size(); j++)
        {
            const Point2f &p1 = getCenter(all_targets[i]);
            const Point2f &p2 = getCenter(all_targets[j]);
            Vec2f direction = p2 - p1;
            Vec2f vertical_direction = {direction[1], -1 * direction[0]};
            Point2f mid_point = (p1 + p2) / 2;
            Vec4f mid_line = {vertical_direction[0], vertical_direction[1], mid_point.x, mid_point.y};
            double target_accuracy = 1.0;
            target_accuracy *= all_targets[i]->getActiveFlag() ? rune_detector_param.ACTIVE_TARGET_ACCURACY : rune_detector_param.INACTIVE_TARGET_ACCURACY;
            target_accuracy *= all_targets[j]->getActiveFlag() ? rune_detector_param.ACTIVE_TARGET_ACCURACY : rune_detector_param.INACTIVE_TARGET_ACCURACY;
            for (auto &center : centers)
            {
                float distance = getDist(mid_line, getCenter(center));
                float distance_ratio = distance / max(center->getImageCache().getWidth(), center->getImageCache().getHeight());
                if (distance_ratio > rune_detector_filter_center_param.MAX_TARGET_MID_LINE_DISTANCE_RATIO)
                {
                    // 弹出警告，显示 distance_ratio
                    center_accuracy[center] *= (1 - target_accuracy);
                }
            }
        }
        // --------------------距离判断-----------------------
        for (auto &center : centers)
        {
            float distance = getDist(getCenter(all_targets[i]), getCenter(center));
            float distance_ratio = distance / max(all_targets[i]->getImageCache().getWidth(), all_targets[i]->getImageCache().getHeight());
            if (distance_ratio > rune_detector_filter_center_param.MAX_TARGET_DISTANCE_RATIO)
            {
                center_accuracy[center] *= (1 - all_targets[i]->getActiveFlag() ? rune_detector_param.ACTIVE_TARGET_ACCURACY : rune_detector_param.INACTIVE_TARGET_ACCURACY);
            }
            if (distance_ratio < rune_detector_filter_center_param.MIN_TARGET_DISTANCE_RATIO)
            {
                center_accuracy[center] *= (1 - all_targets[i]->getActiveFlag() ? rune_detector_param.ACTIVE_TARGET_ACCURACY : rune_detector_param.INACTIVE_TARGET_ACCURACY);
            }
        }
    }

    // 通过神符扇叶筛选
    // 删除
    for (auto &fan : inactive_fans)
    {
        // 计算扇叶方向所在直线
        Point2f fan_center = getCenter(fan);
        Point2f fan_direction = fan->getImageCache().getDirection();
        Vec4f fan_line{fan_direction.x, fan_direction.y, fan_center.x, fan_center.y};
        // 利用神符中心与扇叶所成直线与扇叶方向的的最大夹角来筛选
        for (auto &center : centers)
        {
            // 获取扇叶到中心的向量
            Point2f fan_to_center = getCenter(center) - fan_center;
            // 若扇叶方向与扇叶到中心的向量夹角大于 30 度，则正确率降低
            float angle = getVectorMinAngle(fan_direction, fan_to_center, DEG);
            if (angle > 90)
                angle = 180 - angle;
            if (angle > rune_detector_filter_center_param.MAX_FAN_ANGLE)
            {
                center_accuracy[center] *= (1 - rune_detector_param.INACTIVE_FAN_ACCURACY);
            }
        }

        // 利用 神符中心到扇叶中轴线的距离来筛选
        for (auto &center : centers)
        {
            float distance = getDist(fan_line, getCenter(center));
            float distance_ratio = distance / max(fan->getImageCache().getWidth(), fan->getImageCache().getHeight());
            if (distance_ratio > rune_detector_filter_center_param.MAX_FAN_LINE_DISTANCE_RATIO)
            {
                center_accuracy[center] *= (1 - rune_detector_param.INACTIVE_FAN_ACCURACY);
            }
        }
        // 利用神符中心到扇叶中心的距离比例来筛选
        for (auto &center : centers)
        {
            float distance = getDist(getCenter(fan), getCenter(center));
            float distance_ratio = distance / max(fan->getImageCache().getWidth(), fan->getImageCache().getHeight());
            if (distance_ratio > rune_detector_filter_center_param.MAX_FAN_DISTANCE_RATIO)
            {
                center_accuracy[center] *= (1 - rune_detector_param.INACTIVE_FAN_ACCURACY);
            }
        }
    }
    // 通过已激活扇叶筛选
    for (auto &fan : active_fans)
    {
        // 计算扇叶方向所在直线
        Point2f fan_center = getCenter(fan);
        Point2f fan_direction = fan->getImageCache().getDirection();
        Vec4f fan_line{fan_direction.x, fan_direction.y, fan_center.x, fan_center.y};
        for (auto &center : centers)
        {
            Point2f fan_to_center = getCenter(center) - fan_center;
            float angle = getVectorMinAngle(fan_direction, fan_to_center, DEG);
            if (angle > 90)
                angle = 180 - angle;
            if (angle > rune_detector_filter_center_param.MAX_FAN_ANGLE)
            {
                center_accuracy[center] *= (1 - rune_detector_param.ACTIVE_FAN_ACCURACY);
            }
        }
        for (auto &center : centers)
        {
            Point2f fan_to_center = getCenter(center) - fan_center;
            float angle = getVectorMinAngle(fan_direction, fan_to_center, DEG);
            if (angle > 90)
                angle = 180 - angle;
            if (angle > rune_detector_filter_center_param.MAX_FAN_ANGLE)
            {
                center_accuracy[center] *= (1 - rune_detector_param.ACTIVE_FAN_ACCURACY);
            }
        }
        for (auto &center : centers)
        {
            Point2f fan_to_center = getCenter(center) - fan_center;
            if ((fan_to_center.x * fan_direction.x + fan_to_center.y * fan_direction.y) < 0)
            {
                center_accuracy[center] *= (1 - rune_detector_param.ACTIVE_FAN_ACCURACY);
            }
        }
        for (auto &center : centers)
        {
            float distance = getDist(getCenter(fan), getCenter(center));
            float distance_ratio = distance / max(fan->getImageCache().getWidth(), fan->getImageCache().getHeight());
            if (distance_ratio > rune_detector_filter_center_param.MAX_FAN_DISTANCE_RATIO)
            {
                center_accuracy[center] *= (1 - rune_detector_param.ACTIVE_FAN_ACCURACY);
            }
        }
    }
    vector<tuple<vector<Point2f>, double>> contour_accuracy; // [轮廓 : 该轮廓对应的正确率]
    for (auto &target : inactive_targets)
    {
        vector<Point2f> rect_points(4);
        getRotatedRect(target).points(rect_points.data());
        contour_accuracy.push_back({rect_points, rune_detector_param.INACTIVE_TARGET_ACCURACY});
    }
    for (auto &target : active_targets)
    {
        vector<Point2f> rect_points(4);
        getRotatedRect(target).points(rect_points.data());
        contour_accuracy.push_back({rect_points, rune_detector_param.ACTIVE_TARGET_ACCURACY});
    }
    for (auto &fan : inactive_fans)
    {
        vector<Point2f> rect_points(4);
        getRotatedRect(fan).points(rect_points.data());
        contour_accuracy.push_back({rect_points, rune_detector_param.INACTIVE_FAN_ACCURACY});
    }
    for (auto &fan : active_fans)
    {
        vector<Point2f> rect_points(4);
        getRotatedRect(fan).points(rect_points.data());
        contour_accuracy.push_back({rect_points, rune_detector_param.ACTIVE_FAN_ACCURACY});
    }
    // 更新神符中心的正确率
    for (auto &[contour, accuracy] : contour_accuracy)
    {
        for (auto &center : centers)
        {
            if (pointPolygonTest(contour, getCenter(center), false) > 0)
                center_accuracy[center] *= (1 - accuracy);
        }
    }
    // 删除正确率过低的神符中心
    centers.erase(remove_if(centers.begin(), centers.end(), [&](const auto &center)
                            { return center_accuracy[center] < rune_detector_param.MIN_CENTER_ACCURACY; }),
                  centers.end());

    if (centers.empty())
        return false;

    // 选择正确率最高的神符中心
    auto [best_center, accuracy] = *max_element(center_accuracy.begin(), center_accuracy.end(), [](const auto &lhs, const auto &rhs)
                                                { return lhs.second < rhs.second; });

    centers.clear();
    centers.push_back(best_center);
    return true;
}

bool RuneDetector::filterActiveFanIncomplete(std::vector<FeatureNode_ptr> &fans_Incomplete,
                                             const std::vector<FeatureNode_ptr> &inactive_targets,
                                             const std::vector<FeatureNode_ptr> &active_targets,
                                             const std::vector<FeatureNode_ptr> &inactive_fans,
                                             const std::vector<FeatureNode_ptr> &active_fans,
                                             const FeatureNode_ptr &best_center)
{
    if (fans_Incomplete.empty())
        return false;

    unordered_set<FeatureNode_ptr> filtered_fans;

    // 利用已激活靶心筛选
    if (!active_targets.empty())
    {
        for (auto &fan : fans_Incomplete)
        {
            auto min_target = *min_element(active_targets.begin(), active_targets.end(), [&fan](const FeatureNode_ptr &t1, const FeatureNode_ptr &t2)
                                           { return getDist(getCenter(fan), getCenter(t1)) < getDist(getCenter(fan), getCenter(t2)); });

            Point2f target_to_fan = getCenter(fan) - getCenter(min_target);
            Point2f fan_direction = fan->getImageCache().getDirection();
            float delta_angle = getVectorMinAngle(target_to_fan, fan_direction, DEG);

            float distance = getDist(getCenter(fan), getCenter(min_target));
            if (delta_angle > 20)
                continue;
            if (distance > std::max(fan->getImageCache().getWidth(), fan->getImageCache().getHeight()))
                continue;

            filtered_fans.insert(fan);
        }
    }

    // 利用神符中心筛选
    if (best_center)
    {
        for (auto &fan : fans_Incomplete)
        {
            Point2f v1 = getCenter(best_center) - getCenter(fan);
            Point2f v2 = fan->getImageCache().getDirection();
            float delta_angle = getVectorMinAngle(v1, v2, DEG);
            if (delta_angle > 45)
                continue;

            float distance = getDist(getCenter(fan), getCenter(best_center));
            if (distance > std::max(fan->getImageCache().getWidth(), fan->getImageCache().getHeight()))
                continue;

            filtered_fans.insert(fan);
        }
    }

    // 删除重叠扇叶
    for (const auto &fan1 : fans_Incomplete)
    {
        if (filtered_fans.find(fan1) == filtered_fans.end())
            continue;
        for (const auto &fan2 : fans_Incomplete)
        {
            if (fan1 == fan2)
                continue;
            if (filtered_fans.find(fan2) == filtered_fans.end())
                continue;

            float distance = getDist(getCenter(fan1), getCenter(fan2));
            float min_threshold = (std::max(fan1->getImageCache().getWidth(), fan1->getImageCache().getHeight()) +
                                   std::max(fan2->getImageCache().getWidth(), fan2->getImageCache().getHeight())) /
                                  4.0f;
            if (distance < min_threshold)
            {
                if (RuneFan::cast(fan1)->getError() > RuneFan::cast(fan2)->getError())
                    filtered_fans.erase(fan1);
                else
                    filtered_fans.erase(fan2);
            }
        }
    }

    fans_Incomplete.clear();
    fans_Incomplete.insert(fans_Incomplete.end(), filtered_fans.begin(), filtered_fans.end());

    return true;
}

bool RuneDetector::filterFarContours(const std::vector<Contour_cptr> &contours,
                                     const std::vector<FeatureNode_ptr> &targets,
                                     const FeatureNode_ptr &center,
                                     const std::vector<FeatureNode_ptr> &fans,
                                     const std::unordered_set<size_t> &mask,
                                     std::unordered_set<size_t> &far_contour_idxs)
{
    // 判空
    if (contours.empty())
        return false;
    if (!center)
        return false;
    if (targets.empty() && fans.empty())
        return false;

    // 计算所有特征到中心的最大距离
    float max_distance = 0.f;
    for (const auto &target : targets)
        max_distance = std::max(max_distance, getDist(getCenter(target), getCenter(center)));
    for (const auto &fan : fans)
        max_distance = std::max(max_distance, getDist(getCenter(fan), getCenter(center)));

    if (max_distance == 0.f)
        return false;

    // 设置最大距离阈值
    float max_distance_threshold = max_distance * rune_detector_param.MAX_DISTANCE_RATIO;

    // 筛选距离神符中心过远的轮廓
    for (size_t i = 0; i < contours.size(); ++i)
    {
        if (mask.find(i) != mask.end())
            continue;
        Point2f contour_center = contours[i]->center();
        float distance = getDist(contour_center, getCenter(center));
        if (distance > max_distance_threshold)
            far_contour_idxs.insert(i);
    }

    return true;
}

bool RuneDetector::getRotateCenter(const std::vector<FeatureNode_ptr> &targets, const std::vector<FeatureNode_ptr> &fans, cv::Point2f &rotate_center)
{
    // 判空
    if (targets.empty())
        return false;
    if (fans.empty())
        return false;

    // 获取所有特征的方向射线
    std::vector<cv::Vec4f> direction_lines;
    for (const auto &target : targets)
    {
        auto rune_target = RuneTarget::cast(target);
        if (rune_target && rune_target->getImageCache().isSetDirection())
        {
            cv::Vec4f line{getCenter(rune_target).x, getCenter(rune_target).y,
                           getCenter(rune_target).x + rune_target->getImageCache().getDirection().x,
                           getCenter(rune_target).y + rune_target->getImageCache().getDirection().y};
            direction_lines.push_back(line);
        }
    }
    for (const auto &fan : fans)
    {
        auto rune_fan = RuneFan::cast(fan);
        if (rune_fan && rune_fan->getImageCache().isSetDirection())
        {
            cv::Vec4f line{getCenter(rune_fan).x, getCenter(rune_fan).y,
                           getCenter(rune_fan).x + rune_fan->getImageCache().getDirection().x,
                           getCenter(rune_fan).y + rune_fan->getImageCache().getDirection().y};
            direction_lines.push_back(line);
        }
    }
    std::vector<cv::Point2f> cross_points;
    for (size_t i = 0; i < direction_lines.size(); ++i)
    {
        for (size_t j = i + 1; j < direction_lines.size(); ++j)
        {
            cv::Point2f cross_point = getCrossPoint(direction_lines[i], direction_lines[j]);
            cross_points.push_back(cross_point);
        }
    }
    if (cross_points.empty())
        return false;

    // 获取所有特征的方向射线的交点的平均值
    rotate_center = std::accumulate(cross_points.begin(), cross_points.end(), cv::Point2f(0, 0)) / static_cast<float>(cross_points.size());

    return true;
}

std::vector<std::tuple<FeatureNode_ptr,
                       FeatureNode_ptr,
                       FeatureNode_ptr>>
RuneDetector::getMatchedFeature(
    const std::vector<FeatureNode_ptr> &p_targets,
    const FeatureNode_ptr &p_center,
    const std::vector<FeatureNode_ptr> &p_fans)
{
    vector<tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>> matched_feature{};
    unordered_set<FeatureNode_ptr> pending_targets(p_targets.begin(), p_targets.end());
    unordered_set<FeatureNode_ptr> pending_fans(p_fans.begin(), p_fans.end());

    // 对每个扇叶、匹配其对应的靶心
    for (auto &fan : p_fans)
    {
        if (pending_targets.empty()) // 如果靶心为空，退出
            break;
        map<FeatureNode_ptr, float> delta_angles{};
        for (auto target : pending_targets)
        {
            delta_angles[target] = getVectorMinAngle(-1 * fan->getImageCache().getDirection(), getCenter(target) - getCenter(fan), DEG);
        }
        // 找到最小角度差的靶心
        auto [target, min_angle] = *min_element(delta_angles.begin(), delta_angles.end(), [](const pair<FeatureNode_ptr, float> &lhs, const pair<FeatureNode_ptr, float> &rhs)
                                                { return lhs.second < rhs.second; });
        // 如果角度差小于阈值，则认为匹配成功
        if (min_angle < 20)
        {
            if (target == nullptr || fan == nullptr)
                VC_THROW_ERROR("target or fan is nullptr");
            matched_feature.push_back({target, p_center, fan});
            // 匹配成功，将扇叶和靶心从未匹配集合中移除
            pending_targets.erase(target);
            pending_fans.erase(fan);
        }
    }
    for (auto &match_target : p_targets)
    {
        if (pending_targets.find(match_target) == pending_targets.end())
            continue;
        Point2f direction_1 = getCenter(match_target) - getCenter(p_center);
        float min_delta_angle = 180.f;
        for (auto &[target, center, fan] : matched_feature)
        {
            Point2f direction_2{};
            if (target)
                direction_2 = getCenter(target) - getCenter(center);
            else if (fan)
                direction_2 = getCenter(fan) - getCenter(center);
            else
            {
                VC_THROW_ERROR("target and fan are both nullptr of the \"matched_feature\"");
            }
            min_delta_angle = std::min(min_delta_angle, getVectorMinAngle(direction_1, direction_2, DEG));
        }
        if (min_delta_angle > rune_detector_param.MIN_FORCE_CONSTRUCT_ANGLE)
        {
            matched_feature.emplace_back(match_target, p_center, nullptr);
            pending_targets.erase(match_target);
        }
    }
    for (auto &match_fan : p_fans)
    {
        if (pending_fans.find(match_fan) == pending_fans.end())
            continue;
        Point2f direction_1 = getCenter(match_fan) - getCenter(p_center);
        float min_delta_angle = 180.f;
        for (auto &[target, center, fan] : matched_feature)
        {
            Point2f direction_2{};
            if (target)
                direction_2 = getCenter(target) - getCenter(center);
            else if (fan)
                direction_2 = getCenter(fan) - getCenter(center);
            else
            {
                VC_THROW_ERROR("target and fan are both nullptr of the \"matched_feature\"");
            }
            min_delta_angle = std::min(min_delta_angle, getVectorMinAngle(direction_1, direction_2, DEG));
        }
        if (min_delta_angle > rune_detector_param.MIN_FORCE_CONSTRUCT_ANGLE)
        {
            matched_feature.emplace_back(nullptr, p_center, match_fan);
            pending_fans.erase(match_fan);
        }
    }

    return matched_feature;
}
