#include "vc/detector/rune_detector.h"
#include "vc/detector/rune_detector_param.h"
#include "vc/feature/rune_tracker.h"

using namespace std;
using namespace cv;

inline Point2f getCenter(const FeatureNode_cptr &feature)
{
    if (!feature)
        return Point2f(0.f, 0.f);
    return feature->getImageCache().getCenter();
}

bool RuneDetector::match(const vector<FeatureNode_ptr> &combos, vector<FeatureNode_ptr> &trackers, bool is_vanish_update)
{
    // 异常中断
    if (combos.size() != 5)
    {
        VC_ERROR_INFO("神符组合体数量不为5");
    }

    if (trackers.empty())
    {
        // 为每个识别到的 rune 创建一个 tracker
        for (const auto &p_combo : combos)
        {
            auto tracker = RuneTracker::make_feature();
            tracker->update(p_combo, getTick(), getGyroData());
            trackers.emplace_back(std::move(tracker));
        }
    }

    if (trackers.size() != 5)
    {
        VC_ERROR_INFO("追踪器数量不为5");
    }
    
    // 计算代价矩阵
    vector<vector<float>> cost_matrix(5, vector<float>(5, 0.f));
    for (size_t i = 0; i < 5; i++)
    {
        for (size_t j = 0; j < 5; j++)
        {
            Point2f pos1 = getCenter(combos[i]);
            Point2f pos2 = getCenter(trackers[j]);
            cost_matrix[i][j] = pow(getDist(pos1, pos2), 2);
        }
    }

    // 寻找最小代价
    vector<int> match(5, -1);
    vector<bool> combo_vis(5, false);
    float min_cost = numeric_limits<float>::max(); // 初始化最小代价
    vector<int> current_match(5);

    function<void(int, float)> dfs = [&](int u, float sum_cost)
    {
        if (u == 5)
        {
            if (sum_cost < min_cost)
            {
                min_cost = sum_cost;
                match = current_match;
            }
            return;
        }
        for (int i = 0; i < 5; i++)
        {
            if (!combo_vis[i])
            {
                combo_vis[i] = true;
                current_match[u] = i; // 记录追踪器u匹配组合体i
                dfs(u + 1, sum_cost + cost_matrix[u][i]);
                combo_vis[i] = false;
            }
        }
    };

    dfs(0, 0.f);
    vector<Point2f> tracker_centers(5);
    for (size_t i = 0; i < 5; i++)
    {
        tracker_centers[i] = getCenter(trackers[match[i]]);
    }
    RotatedRect rect = minAreaRect(tracker_centers);
    // 设置追踪器的最大匹配偏移
    float max_offset = min(rect.size.width, rect.size.height) * 0.3f;
    for (size_t i = 0; i < 5; i++)
    {
        RuneTracker::cast(trackers[i])->update(combos[match[i]], getTick(), getGyroData());
        auto rune_tracker = RuneTracker::cast(trackers[i]);
        if (is_vanish_update)
        {
            rune_tracker->updateVisible(false);
        }
        else
        {
            rune_tracker->updateVisible(true);
        }
    }

    return true;
}
