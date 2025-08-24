#include "vc/detector/rune_detector.h"
#include "vc/detector/rune_detector_param.h"
#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_target.h"
#include "vc/feature/rune_center.h"
#include "vc/feature/rune_fan_inactive.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/core/debug_tools.h"

using namespace std;
using namespace cv;

inline Point2f getCenter(const FeatureNode_cptr &feature)
{
    if (!feature)
        return Point2f(0.f, 0.f);
    return feature->getImageCache().getCenter();
}

inline Point2f getDirection(const FeatureNode_cptr &feature)
{
    if (!feature)
        return Point2f(0.f, 0.f);
    return feature->getImageCache().getDirection();
}

/**
 * @brief 判断是否存在匹配异常
 *
 * @param runes 神符组
 */
inline bool isMatchError(const std::vector<RuneFeatureCombo> &runes)
{
    for (auto &[target, center, fan] : runes)
    {
        if ((target == nullptr && fan == nullptr) || center == nullptr)
            return true;

        if (target != nullptr && fan != nullptr)
        {
            if (RuneTarget::cast(target)->getActiveFlag() != RuneFan::cast(fan)->getActiveFlag())
                return true;
        }
    }
    return false;
}

/**
 * @brief 通过未激活扇叶强制构造神符中心
 */
FeatureNode_ptr forceMakeCenterByInactiveFan(vector<FeatureNode_ptr> &inactive_fans, const vector<FeatureNode_ptr> &inactive_targets)
{
    if (inactive_fans.empty() || inactive_targets.empty())
        return nullptr;
    vector<FeatureNode_ptr> rune_centers{};
    vector<FeatureNode_ptr> in_targets(inactive_targets.begin(), inactive_targets.end());
    vector<FeatureNode_ptr> in_fans(inactive_fans.begin(), inactive_fans.end());
    for (auto &fan : in_fans)
    {
        auto end_arrow_contour = RuneFanInactive::getEndArrowContour(fan, to_const(in_targets));
        if (end_arrow_contour == nullptr)
            continue;
        auto rune_center = RuneCenter::make_feature(end_arrow_contour, {});
        if (rune_center == nullptr)
            continue;

        rune_centers.push_back(rune_center);
    }
    // 更新 inactive_fans
    inactive_fans.clear();
    for (auto &fan : in_fans)
        inactive_fans.push_back(RuneFan::cast(fan));

    return !rune_centers.empty() ? rune_centers[0] : nullptr;
}

/**
 * @brief 强制构造神符中心
 *
 * @param targets 靶心
 * @param fans 扇叶
 *
 * @return RuneC_ptr 神符中心
 *
 * @note 1. 利用靶心和扇叶的几何关系，强制构造神符中心
 *       2. 强制构造的神符中心会和靶心、扇叶处于同一平面上,位姿错误，不可用于pnp解算
 *       3. 仅用于临时测试使用
 */
FeatureNode_ptr forceMakeCenter(const vector<FeatureNode_ptr> &inactive_targets, const vector<FeatureNode_ptr> &active_targets,
                                vector<FeatureNode_ptr> &inactive_fans, const vector<FeatureNode_ptr> &active_fans)
{
    // 尝试通过未激活扇叶构造神符中心
    auto rune_center = forceMakeCenterByInactiveFan(inactive_fans, inactive_targets);
    if (rune_center != nullptr)
        return rune_center;

    vector<FeatureNode_ptr> targets{};
    vector<FeatureNode_ptr> fans{};
    if (!inactive_targets.empty())
        targets.insert(targets.end(), inactive_targets.begin(), inactive_targets.end());
    if (!active_targets.empty())
        targets.insert(targets.end(), active_targets.begin(), active_targets.end());
    if (!inactive_fans.empty())
        fans.insert(fans.end(), inactive_fans.begin(), inactive_fans.end());
    if (!active_fans.empty())
        fans.insert(fans.end(), active_fans.begin(), active_fans.end());

    if (targets.empty() && fans.empty())
        return nullptr;

    std::unique_ptr<Point2f> targets_center{nullptr};
    std::unique_ptr<Point2f> fans_center{nullptr};

    if (targets.size() >= 3)
    {
        // 进行外接圆拟合
        vector<Point> points{};
        for (auto &target : targets)
        {
            points.emplace_back(static_cast<Point>(getCenter(target)));
        }
        auto contour_temp = ContourWrapper<int>::make_contour(move(points));
        auto [center, radius] = contour_temp->fittedCircle();
        targets_center = std::make_unique<Point2f>(center.x, center.y);
    }
    if (fans.size() >= 2)
    {
        // 交界点集
        vector<Point2f> fans_intersection{};
        for (size_t i = 0; i < fans.size() - 1; i++)
        {
            for (size_t j = i + 1; j < fans.size(); j++)
            {
                Point2f p1 = getCenter(fans[i]);
                Point2f p2 = getCenter(fans[j]);
                Point2f v1 = getDirection(fans[i]);
                Point2f v2 = getDirection(fans[j]);
                Vec4f line1 = Vec4f(v1.x, v1.y, p1.x, p1.y);
                Vec4f line2 = Vec4f(v2.x, v2.y, p2.x, p2.y);
                // 计算交点
                Point2f intersection = getLineIntersection(line1, line2);
                if (intersection == cv::Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()))
                {
                    continue;
                }
                fans_intersection.emplace_back(intersection);
            }
        }
        if (!fans_intersection.empty())
        {
            // 计算平均点
            Point2f sum{};
            for (const auto &point : fans_intersection)
            {
                sum += point;
            }
            fans_center = std::make_unique<Point2f>(sum.x / static_cast<float>(fans_intersection.size()), sum.y / static_cast<float>(fans_intersection.size()));
        }
    }
    Point2f center{0, 0};
    if (targets_center == nullptr && fans_center == nullptr)
        return nullptr;
    if (targets_center == nullptr)
        center = *fans_center;
    else if (fans_center == nullptr)
        center = *targets_center;
    else
        center = ((*targets_center) * static_cast<float>(targets.size()) + (*fans_center) * static_cast<float>(fans.size())) /
                 static_cast<float>(targets.size() + fans.size());

    // 创建神符中心
    return RuneCenter::make_feature(center);
}

void RuneDetector::extractRuneFeatures(vector<FeatureNode_ptr> &targets_inactive, vector<FeatureNode_ptr> &targets_active, vector<FeatureNode_ptr> &fans_inactive, vector<FeatureNode_ptr> &fans_active, vector<FeatureNode_ptr> &centers, const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, unordered_set<size_t> &continue_idx)
{
    {
        unordered_map<FeatureNode_cptr, unordered_set<size_t>> used_idxs{};
        RuneTarget::find_inactive_targets(targets_inactive, contours, hierarchy, continue_idx, used_idxs);
        filterInactiveTarget(targets_inactive);
        for (const auto &target : targets_inactive)
            continue_idx.insert(used_idxs[target].begin(), used_idxs[target].end());
    }
    {
        unordered_map<FeatureNode_cptr, unordered_set<size_t>> used_idxs{};
        RuneTarget::find_active_targets(targets_active, contours, hierarchy, continue_idx, used_idxs);
        filterActiveTarget(targets_active, targets_inactive);
        for (const auto &target : targets_active)
            continue_idx.insert(used_idxs[target].begin(), used_idxs[target].end());
    }
    {
        unordered_map<FeatureNode_cptr, unordered_set<size_t>> used_idxs{};
        RuneFan::find_active_fans(fans_active, contours, hierarchy, continue_idx, used_idxs);
        for (const auto &fan : fans_active)
            continue_idx.insert(used_idxs[fan].begin(), used_idxs[fan].end());
    }
    {
        unordered_map<FeatureNode_cptr, unordered_set<size_t>> used_idxs{};
        vector<FeatureNode_cptr> targets_inactive_temp{};
        targets_inactive_temp.insert(targets_inactive_temp.end(), targets_inactive.begin(), targets_inactive.end());
        RuneFan::find_inactive_fans(fans_inactive, contours, hierarchy, continue_idx, used_idxs, targets_inactive_temp);
        filterInactiveFan(fans_inactive, targets_inactive, targets_active, fans_active);
        for (const auto &fan : fans_inactive)
            continue_idx.insert(used_idxs[fan].begin(), used_idxs[fan].end());
    }
    {
        unordered_map<FeatureNode_cptr, unordered_set<size_t>> used_idxs{};
        RuneCenter::find(centers, contours, hierarchy, continue_idx, used_idxs);
        filterCenter(centers, targets_inactive, targets_active, fans_inactive, fans_active);
        for (const auto &center : centers)
            continue_idx.insert(used_idxs[center].begin(), used_idxs[center].end());
    }
}

bool RuneDetector::findFeatures(Mat src, vector<FeatureNode_cptr> &features, std::vector<RuneFeatureCombo> &matched_features)
{

    vector<Contour_cptr> contours{};
    vector<Vec4i> hierarchy; // 轮廓等级向量
    // 神符轮廓识别
    findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    // vector<Contour_ptr> contours(contours_temp.begin(), contours_temp.end()); // 轮廓二维向量
    for (size_t i = 0; i < contours.size(); i++) // 删除面积过小的轮廓
    {
        double area = contours[i]->area();
        if (area < rune_detector_param.MIN_CONTOUR_AREA || area > rune_detector_param.MAX_CONTOUR_AREA)
        {
            deleteContour(contours, hierarchy, static_cast<int>(i));
            i--;
        }
    }
    if (contours.empty() || hierarchy.empty())
    {
        return false;
    }
    vector<FeatureNode_ptr> targets_active;   // 神符已激活靶心向量
    vector<FeatureNode_ptr> targets_inactive; // 神符未激活靶心向量
    vector<FeatureNode_ptr> fans_active;      // 神符已激活扇叶向量
    vector<FeatureNode_ptr> fans_inactive;    // 神符未激活扇叶向量
    vector<FeatureNode_ptr> centers;          // 神符旋转中心向量
    unordered_set<size_t> continue_idx{}; // 不进行识别的轮廓下标集合

    extractRuneFeatures(targets_inactive, targets_active, fans_inactive, fans_active, centers, contours, hierarchy, continue_idx);

    if (targets_inactive.empty() && targets_active.empty() && fans_inactive.empty() && fans_active.empty()) // 神符特征为空、放弃构建
        return false;
    // 判断是否需要强制构造神符中心
    bool is_force_make_center = [&]() -> bool
    {
        if (centers.empty())
            return true; // 神符中心为空、需要强制构造
        if (!rune_detector_param.ENABLE_CENTER_FORCE_CONSTRUCT_WINDOW)
        {
            return false; // 不需要强制构造神符中心
        }

        // 若识别到的神符中心位于图像视野边缘位置，则认为识别结果不可靠，需要强制构造神符中心
        // 神符中心不可靠的范围比例
        double rune_center_unreliable_ratio = rune_detector_param.CENTER_FORCE_CONSTRUCT_WINDOW_RATIO;
        int left_x = static_cast<int>(src.cols * rune_center_unreliable_ratio / 2.0);
        int right_x = src.cols - left_x;
        int top_y = static_cast<int>(src.rows * rune_center_unreliable_ratio / 2.0);
        int bottom_y = src.rows - top_y;
        for (const auto &center : centers)
        {
            Point2f center_pos = getCenter(center);
            if (center_pos.x < left_x || center_pos.x > right_x || center_pos.y < top_y || center_pos.y > bottom_y)
            {
                return true; // 神符中心位于图像视野边缘位置
            }
        }
        return false; // 神符中心位于图像视野中心位置
    }();

    do
    {
        if (!is_force_make_center)
        {
            break; // 不需要强制构造神符中心
        }
        // 尝试强制构造神符中心
        auto center = forceMakeCenter(targets_inactive, targets_active, fans_inactive, fans_active);
        if (center)
        {
            centers.clear();
            centers.emplace_back(center);
            break;
        }

        return false; // 无法构造神符中心、放弃构建
    } while (0);

    if (centers.empty())
    {
        return false; // 神符中心为空、放弃构建
    }

    FeatureNode_ptr rune_center = centers[0];                                               // 神符中心
    FeatureNode_ptr rune_inactive_fan = fans_inactive.empty() ? nullptr : fans_inactive[0]; // 未激活扇叶

    RuneFanInactive::correctDirection(rune_inactive_fan, getCenter(rune_center)); // 未激活扇叶的方向矫正
    RuneFanInactive::correctCorners(rune_inactive_fan);                           // 未激活扇叶的角点矫正

    vector<FeatureNode_ptr> rune_targets{};
    vector<FeatureNode_ptr> rune_fans{};
    rune_targets.insert(rune_targets.end(), targets_active.begin(), targets_active.end());
    rune_targets.insert(rune_targets.end(), targets_inactive.begin(), targets_inactive.end());
    rune_fans.insert(rune_fans.end(), fans_active.begin(), fans_active.end());
    rune_fans.insert(rune_fans.end(), fans_inactive.begin(), fans_inactive.end());

    unordered_set<size_t> far_contour_idxs{};
    RuneDetector::filterFarContours(contours, rune_targets, rune_center, rune_fans, continue_idx, far_contour_idxs); // 过滤距离过远的轮廓
    continue_idx.insert(far_contour_idxs.begin(), far_contour_idxs.end());

    Point2f rotate_center{};
    if (RuneDetector::getRotateCenter(rune_targets, rune_fans, rotate_center) == false) // 获取旋转中心
        rotate_center = getCenter(rune_center);                                         // 当旋转中心为空、直接以神符中心为旋转中心

    // 未激活靶心的矫正
    for (auto &target : rune_targets)
    {
        if (RuneTarget::cast(target)->getActiveFlag() == false)
        {
            if (RuneTargetInactive::cast(target)->correct(rotate_center) == false)
            {
            }
        }
    }

    vector<FeatureNode_ptr> fans_active_incomplete; // 未完整的已激活扇叶向量
    unordered_map<FeatureNode_cptr, unordered_set<size_t>> fans_active_incomplete_idxs{};
    RuneFan::find_incomplete_active_fans(fans_active_incomplete, contours, hierarchy, continue_idx, rotate_center, fans_active_incomplete_idxs); // 未完整的已激活扇叶
    filterActiveFanIncomplete(fans_active_incomplete, targets_inactive, targets_active, fans_inactive, fans_active, rune_center);                // 筛选未完整的已激活扇叶
    rune_fans.insert(rune_fans.end(), fans_active_incomplete.begin(), fans_active_incomplete.end());

    auto temp_matched_features = getMatchedFeature(rune_targets, rune_center, rune_fans); // 获取配对后的特征组
    if (isMatchError(temp_matched_features))                                              // 匹配异常
    {
        return false;
    } 

#if FEATURE_NODE_DEBUG
    // 绘制所有特征
    do
    {
        Mat img_show = DebugTools::get()->getImage();
        if (img_show.empty())
            break;
        for (const auto &target : rune_targets)
        {
            target->drawFeature(img_show);
        }
        for (const auto &fan : rune_fans)
        {
            fan->drawFeature(img_show);
        }
        for (const auto &center : centers)
        {
            center->drawFeature(img_show);
        }

    } while (0);
#endif

    matched_features = temp_matched_features;
    // 存入 features
    for (auto [target, center, fan] : temp_matched_features)
    {
        if (target != nullptr)
            features.emplace_back(target);
        if (center != nullptr)
            features.emplace_back(center);
        if (fan != nullptr)
            features.emplace_back(fan);
    }
    return true;
}
