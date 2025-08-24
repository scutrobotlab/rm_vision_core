#include "vc/detector/rune_detector.h"
#include "vc/detector/rune_detector_param.h"
#include "vc/feature/rune_group.h"
#include "vc/core/debug_tools.h"
#include "vc/core/debug_tools/window_auto_layout.h"

using namespace std;
using namespace cv;

/**
 * @brief 更新特征组
 *
 * @param[in] matched_features 匹配好的特征
 * @param[out] features 特征组
 */
static inline void setFeatures(const std::vector<RuneFeatureComboConst> &matched_features, std::vector<FeatureNode_cptr> &features)
{
    // 更新特征组
    for (const auto &[target, center, fan] : matched_features)
    {
        if (target)
            features.emplace_back(target);
        if (center)
            features.emplace_back(center);
        if (fan)
            features.emplace_back(fan);
    }
}

/**
 * @brief 设置序列组的基础属性
 */
inline static void setBaseProperties(std::vector<FeatureNode_ptr> &groups, const GyroData &gyro_data, int64 record_time)
{
    for (auto &group : groups)
    {
        auto rune_group = RuneGroup::cast(group);
        rune_group->getPoseCache().setGyroData(gyro_data);
        rune_group->setTick(record_time);
    }
}



void RuneDetector::detect(DetectorInput &input, DetectorOutput &output)
{
    auto &groups = input.getFeatureNodes();
    auto &input_image = input.getImage();
    auto &tick = input.getTick();
    auto &gyro_data = input.getGyroData();
    auto &color = input.getColor();

    if (groups.size() > 1)
        VC_THROW_ERROR("Size of the argument \"groups\" is greater than 1");
    setInputImage(input_image);
    setTick(tick);
    setGyroData(gyro_data);

    vector<FeatureNode_ptr> current_combos{};   // 当前帧的神符组合体
    vector<FeatureNode_cptr> current_features{}; // 当前帧的所有神符特

    // 初始化存储信息
    if (groups.empty())
        groups.emplace_back(RuneGroup::make_feature());

    setBaseProperties(groups, gyro_data, tick);

    auto rune_group = dynamic_pointer_cast<RuneGroup>(groups.front());
    // 二值化处理图像
    int thesh = color == RED ? rune_detector_param.GRAY_THRESHOLD_RED : rune_detector_param.GRAY_THRESHOLD_BLUE;

    Mat bin;
    binary(input_image, bin, color, thesh);
    WindowAutoLayout::get()->addWindow("Binary Image");
    imshow("Binary Image", bin);


    vector<RuneFeatureCombo> matched_features{}; // 匹配好的特征

    // 更新函数
    auto updateRuneGroup = [&]() -> bool
    {
        // 尝试获取神符中心的估计位置
        // 尝试查找所有的神符特征
        if (!findFeatures(bin, current_features, matched_features))
            return false;

        // cout <<"神符特征查找成功" << endl;
        // 尝试获取PNP解算数据

        PoseNode runeGroup_to_cam;
        if (!getPnpData(runeGroup_to_cam, rune_group, to_const(matched_features)))
            return false;

        // 更新序列组
        if (!rune_group->update(runeGroup_to_cam, gyro_data, tick))
            return false;

        // 尝试获取所有的神符组合体
        if (!getRunes(current_combos, rune_group, to_const(matched_features), rune_group->getPoseCache().getPoseNodes()[CoordFrame::CAMERA]))
            return false;

        // 更新神符中心估计信息

        return true;
    };

    // 掉帧状态下的更新函数
    auto updateRuneGroupVanish = [&]() -> bool
    {
        // 掉帧状态处理
        if (!rune_group->visibilityProcess(false))
            return false;

        PoseNode runeGroup_to_cam;
        if (!rune_group->getCamPnpDataFromPast(runeGroup_to_cam))
            return false;

        // 更新序列组
        if (!rune_group->update(runeGroup_to_cam, gyro_data, tick))
            return false;

        // 尝试获取所有的神符组合体
        if (!getRunes(current_combos, rune_group, rune_group->getLastFrameFeatures(), rune_group->getPoseCache().getPoseNodes()[CoordFrame::CAMERA]))
            return false;

        // 更新特征组
        setFeatures(rune_group->getLastFrameFeatures(), current_features);

        return true;
    };

    // 是否为掉帧更新
    bool is_vanish_update = false;
    if (!updateRuneGroup())
    {
        is_vanish_update = true;
        // 若更新失败，尝试掉帧状态下的更新
        if (!updateRuneGroupVanish())
        {
            is_vanish_update = true;
            // 若掉帧状态下的更新失败，重新构建神符序列组
            groups = {RuneGroup::make_feature()};
            setBaseProperties(groups, gyro_data, tick);
            output.setFeatureNodes(groups);
            output.setValid(false);
            return;
        }
    }
    if (!is_vanish_update)
    {
        // 若更新成功，清空掉帧数量
        rune_group->visibilityProcess(true);
    }
    Mat img_show = DebugTools::get()->getImage();
    if(matched_features.size() > 1)
        rune_group->drawFeature(img_show);

    if (current_combos.empty())
    {
        VC_THROW_ERROR("组合体为空");
    }

    // 匹配
    auto rune_trackers = rune_group->getTrackers();
    if (!match(current_combos, rune_trackers, is_vanish_update))
    {
        rune_trackers.clear();
        match(current_combos, rune_trackers, is_vanish_update);
    }
    rune_group->setTrackers(rune_trackers);

    rune_group->sync(gyro_data, tick);
    
    output.setValid(true);
    output.setFeatureNodes(groups);
}
