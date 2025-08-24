#include "vc/feature/rune_group.h"
#include "vc/feature/rune_group_param.h"
#include "vc/feature/rune_group_filter.h"
#include "vc/feature/rune_combo.h"
#include "vc/feature/rune_target.h"
#include "vc/feature/rune_center.h"
#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_filter_fusion.h"
#include "vc/camera/camera_param.h"
#include "vc/core/debug_tools.h"
#include "vc/core/debug_tools/param_view_manager.h"

using namespace std;
using namespace cv;

inline bool errorValueProcess(const Matx61f &pos)
{
    for (int i = 0; i < 6; i++)
        if (isnan(pos(i)) || isinf(pos(i)) || abs(pos(i)) > 1e5)
            return false;
    return true;
}

inline bool checkPoseDiff(const Matx61f &p1, const Matx61f &p2)
{
    float diff[6];
    for (int i = 0; i < 6; i++)
        diff[i] = p1(i) - p2(i);
    const double limits[6] = {rune_group_param.MAX_X_DEVIATION, rune_group_param.MAX_Y_DEVIATION,
                              rune_group_param.MAX_Z_DEVIATION, rune_group_param.MAX_YAW_DEVIATION,
                              rune_group_param.MAX_PITCH_DEVIATION, rune_group_param.MAX_ROLL_DEVIATION};
    const char *names[6] = {"X", "Y", "Z", "Yaw", "Pitch", "Roll"};
    for (int i = 0; i < 6; i++)
        if (abs(diff[i]) > static_cast<float>(limits[i]))
        {
            VC_WARNING_INFO("%s deviation too large : %f", names[i], diff[i]);
            return false;
        }
    return true;
}

inline bool checkPoseValid(const PoseNode &pose)
{
    auto t = pose.tvec(), r = pose.rvec();
    for (int i = 0; i < 3; i++)
        if (isnan(t(i)) || isnan(r(i)) || isinf(t(i)) || isinf(r(i)))
            return false;
    return true;
}

inline Point2d calculateTargetAngle(const Point3d &p)
{
    float yaw = rad2deg(atan2(p.x, p.z));
    float pitch = rad2deg(atan2(p.y, sqrt(p.x * p.x + p.z * p.z)));
    return {yaw, pitch};
}

bool RuneGroup::checkExtremePose(const PoseNode &r_cam, const PoseNode &r_gyro, const GyroData &gyro)
{
    if (!rune_group_param.ENABLE_EXTREME_VALUE_FILTER)
        return true;
    if (!checkPoseValid(r_cam) || !checkPoseValid(r_gyro))
        return false;

    float dis = cv::norm(r_cam.tvec());
    if (dis > rune_group_param.MAX_DISTANCE || dis < rune_group_param.MIN_DISTANCE)
    {
        VC_WARNING_INFO("Camera distance out of range : %f", dis);
        return false;
    }

    bool gimbal_lock = false;
    Matx61f pos_cam = DataConverter::cvtPos(r_cam.tvec(), r_cam.rvec(), gimbal_lock);
    if (gimbal_lock)
    {
        VC_WARNING_INFO("Camera pose triggers gimbal lock");
        return false;
    }

    float yaw = pos_cam(3);
    if (yaw > rune_group_param.MAX_YAW_DEVIATION_ANGLE || yaw < -rune_group_param.MAX_YAW_DEVIATION_ANGLE)
    {
        VC_WARNING_INFO("Camera yaw angle out of range : %f", yaw);
        return false;
    }

    auto [ry, rp] = calculateTargetAngle(r_cam.tvec());
    if (ry > rune_group_param.MAX_ROTATE_YAW || ry < -rune_group_param.MAX_ROTATE_YAW)
    {
        VC_WARNING_INFO("Camera rotate yaw out of range : %f", ry);
        return false;
    }
    if (rp > rune_group_param.MAX_ROTATE_PITCH || rp < -rune_group_param.MAX_ROTATE_PITCH)
    {
        VC_WARNING_INFO("Camera rotate pitch out of range : %f", rp);
        return false;
    }

    Matx61f pos_gyro = DataConverter::cvtPos(r_gyro.tvec(), r_gyro.rvec(), gimbal_lock);
    if (gimbal_lock)
    {
        VC_WARNING_INFO("Gyro pose triggers gimbal lock");
        return false;
    }

    float pitch = pos_gyro(4);
    if (pitch > rune_group_param.MAX_PITCH_DEVIATION_ANGLE || pitch < -rune_group_param.MAX_PITCH_DEVIATION_ANGLE)
    {
        VC_WARNING_INFO("Gyro pitch angle out of range : %f", pitch);
        return false;
    }

    return true;
}

bool RuneGroup::update(const PoseNode &r_cam_raw, const GyroData &gyro, int64_t tick)
{
    clearPreviousData();
    if (!_filter)
        _filter = RuneFilterFusion::make_filter();

    bool same_frame = isSetLastUpdateTick() && (tick == getLastUpdateTick());
    setLastUpdateTick(tick);

    PoseNode corrected{};
    correctRoll(r_cam_raw, corrected);
    auto cam_to_gyro = isSetCamToGyro() ? getCamToGyro() : calcCamToGyroPnpData(gyro);
    PoseNode r_gyro_raw = corrected + cam_to_gyro;

    bool vanish = getVanishNum() > 0;
    if (!vanish)
        setLastValidPoseGyro(r_gyro_raw);

    bool gimbal = false;
    auto &converters = __data_converters;
    if (converters.find("4") == converters.end())
        converters["4"] = DataConverter::make_converter();
    Matx61f raw_pos = converters["4"]->toFilterForm(r_gyro_raw.tvec(), r_gyro_raw.rvec(), gimbal);

    if (_filter->isValid() && !checkPoseDiff(_filter->getPredict(), raw_pos))
        return false;
    if (!checkExtremePose(r_cam_raw, r_gyro_raw, gyro))
        return false;
    if (!errorValueProcess(raw_pos))
        return false;

    RuneFilterStrategy::FilterInput input{raw_pos, tick, cam_to_gyro, !vanish};
    auto out = _filter->filter(input);
    Matx61f filter_pos = out.filtered_pos;
    auto [tvec, rvec] = DataConverter::toTvecAndRvec(filter_pos);
    PoseNode r_gyro(rvec, tvec);
    updatePnpData(r_gyro, gyro);
    if (!vanish)
        updateCenterEstimation();
    return true;
}

bool RuneGroup::getCamPnpDataFromFilter(PoseNode &rune_to_cam) const
{
    if (!_filter || !_filter->isValid())
        return false;
    auto [tvec, rvec] = DataConverter::toTvecAndRvec(_filter->getPredict());
    PoseNode r_gyro(rvec, tvec);
    rune_to_cam = r_gyro + calcCamToGyroPnpData(getPoseCache().getGyroData()).inv();
    return true;
}

bool RuneGroup::getCamPnpDataFromPast(PoseNode &rune_to_cam) const
{
    PoseNode pos_to_cam;
    if (!getCamPnpDataFromFilter(pos_to_cam))
        return false;

    float roll = 0;
    if (isSetPredictFunc() && getPredictFunc())
    {
        auto &ticks = getHistoryTicks();
        if (ticks.size() >= 2)
        {
            float angle = getPredictFunc()(ticks.front());
            if (!std::isnan(angle) && !std::isinf(angle))
                roll = angle;
        }
    }

    auto cam_to_gyro = calcCamToGyroPnpData(getPoseCache().getGyroData());
    auto pos_to_gyro = pos_to_cam + cam_to_gyro;
    bool gimbal = false;
    auto pos_ = DataConverter::cvtPos(pos_to_gyro.tvec(), pos_to_gyro.rvec(), gimbal);
    pos_(5) = roll;
    auto [tvec, rvec] = DataConverter::toTvecAndRvec(pos_);
    rune_to_cam = PoseNode(rvec, tvec) + cam_to_gyro.inv();
    return true;
}

std::vector<RuneFeatureComboConst> RuneGroup::getLastFrameFeatures() const
{
    std::vector<RuneFeatureComboConst> features;
    for (auto &tracker : getTrackers())
    {
        auto combo = TrackingFeatureNode::cast(tracker)->getHistoryNodes().front();
        if (!combo)
            continue;
        auto rune = RuneCombo::cast(combo);
        if (rune->getRuneType() == RuneType::UNKNOWN || rune->getRuneType() == RuneType::UNSTRUCK)
            continue;
        features.emplace_back(std::make_tuple(
            RuneTarget::cast(rune->getChildFeatures().at(ChildFeatureType::RUNE_TARGET)),
            RuneCenter::cast(rune->getChildFeatures().at(ChildFeatureType::RUNE_CENTER)),
            RuneFan::cast(rune->getChildFeatures().at(ChildFeatureType::RUNE_FAN))));
    }
    return features;
}

bool RuneGroup::visibilityProcess(bool obs)
{
    auto &vn = getVanishNum();
    if (!obs)
    {
        if (++vn > rune_group_param.MAX_VANISH_NUMBER)
            return false;
    }
    else
        vn = 0;
    return true;
}

void RuneGroup::clearPreviousData() { clearCamToGyro(); }

inline bool isFilterDataValid(const Matx61f &pos)
{
    for (int i = 0; i < 6; i++)
        if (isnan(pos(i)) || isinf(pos(i)) || abs(pos(i)) > 1e5)
            return false;
    return true;
}

bool RuneGroup::getCurrentRotateAngle(float &angle) const
{
    Matx61f pos = _filter->getLatestValue();
    float roll = 0;
    if (isFilterDataValid(pos))
        roll = pos(5);
    else
    {
        auto &raw = getRawDatas();
        if (raw.size() >= 2)
            roll = (raw[0] - raw[1]) + raw[0];
        else if (raw.size() == 1)
            roll = raw.front();
    }
    angle = roll;
    return true;
}

void RuneGroup::sync(const GyroData &, int64_t)
{
    float roll;
    getCurrentRotateAngle(roll);
    auto &raw = getRawDatas();
    raw.push_front(roll);
    if (raw.size() > rune_group_param.RAW_DATAS_SIZE)
        raw.pop_back();
    auto &ticks = getHistoryTicks();
    ticks.push_front(getTick());
    if (ticks.size() > rune_group_param.RAW_DATAS_SIZE)
        ticks.pop_back();
}

inline unordered_set<size_t> getAbnormalFrames(const vector<TrackingFeatureNode_cptr> &trackers, size_t range)
{
    unordered_set<size_t> abn;
    vector<size_t> lens(trackers.size());
    for (size_t i = 0; i < trackers.size(); i++)
        lens[i] = TrackingFeatureNode::cast(trackers[i])->getHistoryNodes().size();
    range = min(range, *min_element(lens.begin(), lens.end()));

    deque<vector<RuneType>> types(range, vector<RuneType>(trackers.size(), RuneType::UNKNOWN));
    for (size_t f = 0; f < range; f++)
        for (size_t t = 0; t < trackers.size(); t++)
        {
            auto combo = RuneCombo::cast(TrackingFeatureNode::cast(trackers[t])->getHistoryNodes()[f]);
            types[f][t] = (!combo || combo->getRuneType() == RuneType::UNKNOWN) ? RuneType::UNKNOWN : combo->getRuneType();
        }

    for (size_t f = 0; f < range; f++)
    {
        int cnt = 0;
        for (auto &r : types[f])
            if (r == RuneType::PENDING_STRUCK)
                cnt++;
        if (cnt == 0 || cnt > 1)
            abn.insert(f);
    }

    for (size_t f = 0; f < range - 1; f++)
    {
        if (abn.count(f) || abn.count(f + 1))
            continue;
        int old_cnt = 0, new_cnt = 0;
        for (auto &r : types[f + 1])
            if (r == RuneType::STRUCK)
                old_cnt++;
        for (auto &r : types[f])
            if (r == RuneType::STRUCK)
                new_cnt++;
        if (old_cnt > 0 && new_cnt > 0 && new_cnt < old_cnt)
        {
            abn.insert(f);
            abn.insert(f + 1);
        }
    }
    return abn;
}

bool RuneGroup::isTargetCenterChanged() const
{
    vector<TrackingFeatureNode_cptr> trackers;
    for (auto &t : getTrackers())
        trackers.push_back(TrackingFeatureNode::cast(t));
    if (trackers.size() != 5)
        return false;
    for (auto &t : trackers)
        if (t->getHistoryNodes().size() < 3)
            return false;

    int history_size = 10;
    vector<int> lens;
    for (auto &t : trackers)
        lens.push_back((int)t->getHistoryNodes().size());
    history_size = min(history_size, *min_element(lens.begin(), lens.end()));

    auto abn = getAbnormalFrames(trackers, history_size);
    vector<int> pending_count(trackers.size(), 0);

    for (size_t f = 0; f < history_size; f++)
    {
        if (abn.count(f))
            continue;
        for (size_t t = 0; t < trackers.size(); t++)
        {
            auto combo = RuneCombo::cast(trackers[t]->getHistoryNodes()[f]);
            if (combo && combo->getRuneType() == RuneType::PENDING_STRUCK)
                pending_count[t]++;
        }
    }

    int cnt = 0;
    constexpr int threshold_count = 2;
    for (auto &c : pending_count)
        if (c > threshold_count)
            cnt++;
    return cnt > 1;
}

int findMode(const vector<int> &nums)
{
    unordered_map<int, int> m;
    int mode = nums.empty() ? -1 : nums[0], max_count = 0;
    for (int n : nums)
    {
        if (++m[n] > max_count)
        {
            max_count = m[n];
            mode = n;
        }
    }
    return mode;
}

bool RuneGroup::updateCenterEstimation()
{
    if (getTrackers().empty())
        return false;
    const auto &tracker = TrackingFeatureNode::cast(getTrackers().front());
    if (tracker->getHistoryNodes().empty())
        return false;
    const auto &combo = tracker->getHistoryNodes().front();
    const auto &center = RuneCenter::cast(combo->getChildFeatures().at(ChildFeatureType::RUNE_CENTER));
    auto center_to_gyro = center->getPoseCache().getPoseNodes()[CoordFrame::CAMERA] + getCamToGyro();
    CenterEstimationInfo info{true, center_to_gyro.tvec(), getTick()};
    setCenterEstimationInfo(info);
    return true;
}

std::vector<FeatureNode_ptr> RuneGroup::getTrackers()
{
    std::vector<FeatureNode_ptr> t;
    for (auto &[id, tr] : getChildFeatures())
        t.emplace_back(tr);
    return t;
}

const std::vector<FeatureNode_cptr> RuneGroup::getTrackers() const
{
    std::vector<FeatureNode_cptr> t;
    for (auto &[id, tr] : getChildFeatures())
        t.emplace_back(tr);
    return t;
}

template <class T>
std::string get_shared_id(const std::shared_ptr<T> &sp)
{
    if (!sp)
        return {};
    uintptr_t addr = reinterpret_cast<uintptr_t>(std::shared_ptr<void>(sp).get());
    std::ostringstream oss;
    oss << "obj-" << std::hex << std::setw(sizeof(uintptr_t) * 2) << std::setfill('0') << addr;
    return oss.str();
}

void RuneGroup::setTrackers(const std::vector<FeatureNode_ptr> &trackers)
{
    getChildFeatures().clear();
    for (auto &tr : trackers)
    {
        if (!tr)
            continue;
        auto id = get_shared_id(tr);
        if (id.empty())
            continue;
        getChildFeatures()[id] = tr;
    }
}

inline void drawCube(cv::Mat &img, const PoseNode &p, float x_len, float y_len, float z_len, cv::Scalar c)
{
    float hx = x_len / 2, hy = y_len / 2, hz = z_len / 2;
    vector<Point3f> pts3d = {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz}, {-hx, hy, -hz}, {-hx, -hy, hz}, {hx, -hy, hz}, {hx, hy, hz}, {-hx, hy, hz}};
    vector<Point2f> pts2d;
    projectPoints(pts3d, p.rvec(), p.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, pts2d);
    for (int i = 0; i < 4; i++)
    {
        line(img, pts2d[i], pts2d[(i + 1) % 4], c, 1);
        line(img, pts2d[i + 4], pts2d[(i + 1) % 4 + 4], c, 1);
        line(img, pts2d[i], pts2d[i + 4], c, 1);
    }
}

inline void drawPentagonalPrism(cv::Mat &img, const PoseNode &p,
                                float radius, float height,
                                cv::Scalar c)
{
    float hz = height / 2.0f; // 高度一半
    std::vector<cv::Point3f> pts3d;

    const float angle_step = 2.0f * CV_PI / 5.0f;
    // 下底面 (z = -hz)
    for (int i = 0; i < 5; ++i)
    {
        float angle = i * angle_step - CV_PI / 2.0f; // 起点朝Y轴正方向
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);
        pts3d.emplace_back(x, y, -hz);
    }
    // 上底面 (z = +hz)
    for (int i = 0; i < 5; ++i)
    {
        float angle = i * angle_step - CV_PI / 2.0f;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);
        pts3d.emplace_back(x, y, hz);
    }

    // 投影
    std::vector<cv::Point2f> pts2d;
    projectPoints(pts3d, p.rvec(), p.tvec(),
                  camera_param.cameraMatrix, camera_param.distCoeff, pts2d);

    // 绘制底面、顶面和侧面
    for (int i = 0; i < 5; ++i)
    {
        int next = (i + 1) % 5;
        // 下底面
        cv::line(img, pts2d[i], pts2d[next], c, 1);
        // 上底面
        cv::line(img, pts2d[i + 5], pts2d[next + 5], c, 1);
        // 连接侧边
        cv::line(img, pts2d[i], pts2d[i + 5], c, 1);
    }
}

void RuneGroup::drawFeature(cv::Mat &img, const DrawConfig_cptr &config) const
{
    if (img.empty())
        return;
    auto &pose_info = getPoseCache();
    if (pose_info.getPoseNodes().find(CoordFrame::CAMERA) == pose_info.getPoseNodes().end())
        return;
    auto &p = pose_info.getPoseNodes().at(CoordFrame::CAMERA);
    for (const auto &tracker : getTrackers())
        tracker->drawFeature(img, config);
}
