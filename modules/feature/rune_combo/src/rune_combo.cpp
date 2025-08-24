#include "vc/feature/rune_combo.h"
#include "vc/feature/rune_fan_param.h"
#include "vc/feature/rune_target_param.h"
#include "vc/feature/rune_center_param.h"

#include "vc/feature/rune_target_active.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_fan_active.h"
#include "vc/feature/rune_fan_inactive.h"
#include "vc/feature/rune_center.h"

#include "vc/camera/camera_param.h"


using namespace std;
using namespace cv;

RuneCombo_ptr RuneCombo::make_feature(const PoseNode &pnp_data, const RuneType &type, const GyroData &gyro_data, int64 tick)
{
    //------------------ 获取靶心特征 -------------------
    PoseNode target_pnp_data{};
    FeatureNode_ptr target = nullptr;
    if (type == RuneType::STRUCK)
    {
        target_pnp_data.tvec(pnp_data.tvec() + pnp_data.rmat() * rune_target_param.TRANSLATION);
        target_pnp_data.rvec(rune_target_param.ROTATION * pnp_data.rvec());
        target = RuneTargetActive::make_feature(target_pnp_data);
    }
    else if (type == RuneType::UNSTRUCK || type == RuneType::PENDING_STRUCK)
    {
        target_pnp_data.tvec(pnp_data.tvec() + pnp_data.rmat() * rune_target_param.TRANSLATION);
        target_pnp_data.rvec(rune_target_param.ROTATION * pnp_data.rvec());
        target = RuneTargetInactive::make_feature(target_pnp_data);
    }
    else
    {
        VC_THROW_ERROR("The type is not STRUCK or PENDING_STRUCK or PENDING_UNSTRUCK");
    }
    //------------------ 获取神符中心特征 -------------------
    PoseNode center_pnp_data{};
    center_pnp_data.tvec(pnp_data.tvec() + pnp_data.rmat() * rune_center_param.TRANSLATION);
    center_pnp_data.rvec(rune_center_param.ROTATION * pnp_data.rvec());
    FeatureNode_ptr center = RuneCenter::make_feature(center_pnp_data);
    //------------------ 获取扇叶特征 -------------------
    PoseNode fan_pnp_data{};
    FeatureNode_ptr fan = nullptr;
    if (type == RuneType::STRUCK)
    {
        fan_pnp_data.tvec(pnp_data.tvec() + pnp_data.rmat() * rune_fan_param.ACTIVE_TRANSLATION);
        fan_pnp_data.rvec(rune_fan_param.ACTIVE_ROTATION * pnp_data.rvec());
        fan = RuneFan::make_feature(fan_pnp_data, true);
    }
    else if (type == RuneType::PENDING_STRUCK || type == RuneType::UNSTRUCK)
    {
        fan_pnp_data.tvec(pnp_data.tvec() + pnp_data.rmat() * rune_fan_param.INACTIVE_TRANSLATION);
        fan_pnp_data.rvec(rune_fan_param.INACTIVE_ROTATION * pnp_data.rvec());
        fan = RuneFan::make_feature(fan_pnp_data, false);
    }
    else
    {
        VC_THROW_ERROR("The type is not STRUCK or PENDING_STRUCK or PENDING_UNSTRUCK");
    }

    return make_shared<RuneCombo>(target, center, fan, pnp_data, type, gyro_data, tick);
}

/**
 * @brief 通过三个特征计算神符的角点
 *
 * @param p_target 靶心特征
 * @param p_center 中心特征
 * @param p_fan 扇叶特征
 */
inline vector<Point2f> getRuneCorners(const FeatureNode_cptr &p_target, const FeatureNode_cptr &p_center, const FeatureNode_cptr &p_fan)
{
    // 获取击打区域（靶心）的半径
    float radius = rune_target_param.RADIUS;
    vector<Point3f> corners_3d{};
    corners_3d.emplace_back(0, -radius, 0);
    corners_3d.emplace_back(radius, 0, 0);
    corners_3d.emplace_back(0, radius, 0);
    corners_3d.emplace_back(-radius, 0, 0);

    // 进行重投影
    vector<Point2f> corners_2d{};
    projectPoints(corners_3d, p_target->getPoseCache().getPoseNodes().at(CoordFrame::CAMERA).rvec(),
         p_target->getPoseCache().getPoseNodes().at(CoordFrame::CAMERA).tvec(),
          camera_param.cameraMatrix, camera_param.distCoeff, corners_2d);
    return corners_2d;          
}

RuneCombo::RuneCombo(const FeatureNode_ptr &p_target,
                     const FeatureNode_ptr &p_center,
                     const FeatureNode_ptr &p_fan,
                     const PoseNode &rune_to_cam,
                     const RuneType &type,
                     const GyroData &gyro_data,
                     int64 tick)
{
    // ----------------获取轮廓的最小外接矩形--------------------
    vector<Point> temp_contour{};
    if (p_target != nullptr)
    {
        for(const auto& contour : p_target->getImageCache().getContours())
        {
            temp_contour.insert(temp_contour.end(), contour->points().begin(), contour->points().end());
        }
    }
    if (p_center != nullptr)
    {
        for(const auto& contour : p_center->getImageCache().getContours())
        {
            temp_contour.insert(temp_contour.end(), contour->points().begin(), contour->points().end());
        }
    }
    if (p_fan != nullptr)
    {
        for(const auto& contour : p_fan->getImageCache().getContours())
        {
            temp_contour.insert(temp_contour.end(), contour->points().begin(), contour->points().end());
        }
    }
    RotatedRect rect = minAreaRect(temp_contour);
    auto width = max(rect.size.width, rect.size.height);
    auto height = min(rect.size.width, rect.size.height);
    auto center = p_target->getImageCache().getCenter();     // 神符组合体的中心就是靶心的中心

    auto& image_info = getImageCache();
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCenter(center);

    PoseNode cam_to_joint(camera_param.cam2joint_rmat, camera_param.cam2joint_tvec);
    PoseNode rune_to_joint = rune_to_cam + cam_to_joint;
    Matx33f joint_to_gyro_R = euler2Mat(deg2rad(gyro_data.rotation.yaw), Y) * euler2Mat(deg2rad(-1 * gyro_data.rotation.pitch), X);
    PoseNode joint_to_gyro(joint_to_gyro_R, Vec3f{0, 0, 0});

    PoseNode rune_to_gyro = rune_to_joint + joint_to_gyro;



    auto &pose_info = getPoseCache();
    pose_info.setGyroData(gyro_data);
    pose_info.getPoseNodes()[CoordFrame::CAMERA] = rune_to_cam; // 相机坐标系
    pose_info.getPoseNodes()[CoordFrame::JOINT] = rune_to_joint; // 转轴坐标系
    pose_info.getPoseNodes()[CoordFrame::GYRO] = rune_to_gyro; // 陀螺仪坐标系

    // ------------- 更新神符角度 -------------
    // 获取旋转矩阵
    const auto &R = rune_to_gyro.rmat();
    float r11 = R(0, 0), r21 = R(1, 0);
    float roll = rad2deg(atan2(r21, r11));

    // --------------设置神符组合体的角点--------------------
    auto corners = getRuneCorners(p_target, p_center, p_fan);
    image_info.setCorners(corners);
    // ---------- 设置组合体特征指针 ----------
    auto& child_features = getChildFeatures();
    child_features[FeatureNode::ChildFeatureType::RUNE_TARGET] = p_target;
    child_features[FeatureNode::ChildFeatureType::RUNE_CENTER] = p_center;
    child_features[FeatureNode::ChildFeatureType::RUNE_FAN] = p_fan;
    // ---------- 更新组合体类型信息 ----------
    setRuneType(type);
    setTick(tick);
}
