#include "vc/detector/rune_detector.h"
#include "vc/detector/rune_detector_param.h"
#include "vc/feature/rune_combo.h"
#include "vc/camera/camera_param.h"

#include "vc/feature/rune_target.h"
#include "vc/feature/rune_center.h"
#include "vc/feature/rune_fan.h"

#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_target_active.h"

#include "vc/feature/rune_fan_inactive.h"
#include "vc/feature/rune_fan_active.h"

#include "vc/feature/rune_target_param.h"
#include "vc/feature/rune_center_param.h"
#include "vc/feature/rune_fan_param.h"

using namespace std;
using namespace cv;
inline Point2f getCenter(const FeatureNode_cptr feature)
{
    return feature ? feature->getImageCache().getCenter() : Point2f(0.f, 0.f);
}

inline bool setTempRunePnpData(vector<PoseNode> &pnp_datas, const PoseNode &camera_pnp_data)
{
    if (pnp_datas.size() != 5)
        VC_THROW_ERROR("pnp_datas size != 5");
    for (size_t i = 0; i < 5; i++)
    {
        PoseNode rune_pnp = camera_pnp_data;
        rune_pnp.rotate_z(i * 72.f);
        Vec3f offset(-rune_center_param.TRANSLATION(0), -rune_center_param.TRANSLATION(1), 0);
        Vec3f tvec = rune_pnp.tvec() + rune_pnp.rmat() * offset;
        pnp_datas[i] = PoseNode(rune_pnp.rmat(), tvec);
    }
    return true;
}

inline bool setTempRuneType(vector<RuneType> &types, const vector<PoseNode> &pnp_datas,
                            const vector<tuple<FeatureNode_cptr, FeatureNode_cptr, FeatureNode_cptr>> &features_visible,
                            const FeatureNode_ptr &group)
{
    if (types.size() != 5 || pnp_datas.size() != 5)
        VC_THROW_ERROR("size != 5");
    unordered_set<size_t> pending = {0, 1, 2, 3, 4};
    auto getProj = [&](const Matx31f &src, const PoseNode &p)
    {
        vector<Point2f> proj;
        projectPoints(src, p.rvec(), p.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, proj);
        return proj[0];
    };

    for (auto &grp : features_visible)
    {
        auto [target, center, fan] = grp;
        RuneType type = fan ? (RuneFan::cast(fan)->getActiveFlag() ? RuneType::STRUCK : RuneType::PENDING_STRUCK)
                            : (RuneTarget::cast(target)->getActiveFlag() ? RuneType::STRUCK : RuneType::PENDING_STRUCK);
        int idx = -1;
        float min_dist = FLT_MAX;
        for (auto &n : pending)
        {
            Point2f p1 = target ? getCenter(target) : getCenter(fan);
            Point2f p2 = target ? getProj(rune_target_param.TRANSLATION, pnp_datas[n])
                                : getProj(RuneFan::cast(fan)->getActiveFlag() ? rune_fan_param.ACTIVE_TRANSLATION : rune_fan_param.INACTIVE_TRANSLATION, pnp_datas[n]);
            float d = getDist(p1, p2);
            if (d < min_dist)
            {
                min_dist = d;
                idx = (int)n;
            }
        }
        if (idx >= 0 && idx < 5)
        {
            types[idx] = type;
            pending.erase(idx);
        }
    }
    for (auto &idx : pending)
        types[idx] = RuneType::UNSTRUCK;
    return true;
}

inline bool setAllfeatures(const vector<PoseNode> &pnp_datas, const vector<RuneType> &types,
                           vector<tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>> &rune_features)
{
    if (pnp_datas.size() != 5 || types.size() != 5)
        VC_THROW_ERROR("size != 5");
    rune_features.resize(5);
    for (size_t i = 0; i < 5; i++)
    {
        const PoseNode &rune_to_cam = pnp_datas[i];

        // 靶心
        PoseNode target_to_rune(rune_target_param.ROTATION, rune_target_param.TRANSLATION);
        FeatureNode_ptr target = (types[i] == RuneType::STRUCK) ? static_cast<FeatureNode_ptr>(RuneTargetActive::make_feature(target_to_rune + rune_to_cam)) : static_cast<FeatureNode_ptr>(RuneTargetInactive::make_feature(target_to_rune + rune_to_cam));

        // 中心
        PoseNode center_to_cam = PoseNode(rune_center_param.ROTATION, rune_center_param.TRANSLATION) + rune_to_cam;
        FeatureNode_ptr center = RuneCenter::make_feature(center_to_cam);

        // 扇叶
        PoseNode fan_to_rune = (types[i] == RuneType::STRUCK) ? PoseNode(rune_fan_param.ACTIVE_ROTATION, rune_fan_param.ACTIVE_TRANSLATION) : PoseNode(rune_fan_param.INACTIVE_ROTATION, rune_fan_param.INACTIVE_TRANSLATION);
        FeatureNode_ptr fan = RuneFan::make_feature(fan_to_rune + rune_to_cam, types[i] == RuneType::STRUCK);

        rune_features[i] = {target, center, fan};
        if (!target || !center || !fan)
            return false;
    }
    return true;
}

bool RuneDetector::getRunes(vector<FeatureNode_ptr> &current_combos,
                            const FeatureNode_ptr &group,
                            const vector<RuneFeatureComboConst> &features_visible,
                            const PoseNode &camera_pnp_data) const
{
    if (!group || features_visible.empty())
        return false;

    vector<tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>> pnp_runes(5);
    vector<PoseNode> pnp_datas(5);
    vector<RuneType> pnp_types(5);

    if (!setTempRunePnpData(pnp_datas, camera_pnp_data))
        return false;
    if (!setTempRuneType(pnp_types, pnp_datas, features_visible, group))
        return false;
    if (!setAllfeatures(pnp_datas, pnp_types, pnp_runes))
        return false;

    current_combos.clear();
    for (size_t i = 0; i < 5; i++)
    {
        auto [target, center, fan] = pnp_runes[i];
        if (!target || !center || !fan)
            continue;
        auto rune = RuneCombo::make_feature(pnp_datas[i], pnp_types[i], getGyroData(), getTick());
        if (rune)
            current_combos.push_back(dynamic_pointer_cast<RuneCombo>(rune));
    }
    return true;
}
