#include "vc/feature/rune_group.h"
#include "vc/feature/rune_group_param.h"
#include "vc/feature/rune_group_filter.h"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;


void RuneGroup::updatePnpData(const PoseNode &rune_to_gyro, const GyroData &gyro_data)
{
    cv::Matx33f joint_to_gyro_R = gyroEuler2RotMat(gyro_data.rotation.yaw, gyro_data.rotation.pitch);
    PoseNode joint_to_gyro(joint_to_gyro_R, Vec3f{0, 0, 0});
    PoseNode gyro_to_joint = joint_to_gyro.inv();

    PoseNode cam_to_joint(camera_param.cam2joint_rmat, camera_param.cam2joint_tvec);
    PoseNode joint_to_cam = cam_to_joint.inv();

    auto rune_to_joint = rune_to_gyro + gyro_to_joint;
    auto rune_to_cam = rune_to_joint + joint_to_cam;

    // 更新PNP数据
    auto& pose_nodes = this->getPoseCache().getPoseNodes();
    pose_nodes[CoordFrame::GYRO] = rune_to_gyro;
    pose_nodes[CoordFrame::JOINT] = rune_to_joint;
    pose_nodes[CoordFrame::CAMERA] = rune_to_cam;
    setCamToGyro(cam_to_joint + joint_to_gyro);
}

PoseNode RuneGroup::calcCamToGyroPnpData(const GyroData &gyro_data)
{
    PoseNode cam_to_joint(camera_param.cam2joint_rmat, camera_param.cam2joint_tvec);
    cv::Matx33f joint_to_gyro_R = gyroEuler2RotMat(gyro_data.rotation.yaw, gyro_data.rotation.pitch);
    PoseNode joint_to_gyro(joint_to_gyro_R, Vec3f{0, 0, 0});
    PoseNode cam_to_gyro = cam_to_joint + joint_to_gyro;
    return cam_to_gyro;
}

bool RuneGroup::correctRoll(const PoseNode &raw_cam_pnp_data, PoseNode &corrected_cam_pnp_data)
{
    PoseNode last_raw_cam_pnp{};
    if (!_correntRoll_last_cam_pnp_data_flag)
    {
        _correntRoll_last_cam_pnp_data_flag = true;
        _correntRoll_last_cam_pnp_data = raw_cam_pnp_data;
        corrected_cam_pnp_data = raw_cam_pnp_data;
        return false;
    }
    last_raw_cam_pnp = _correntRoll_last_cam_pnp_data;

    // 计算roll角
    auto &data_converters = __data_converters;

    if (data_converters.find("1") == data_converters.end())
    {
        data_converters["1"] = DataConverter::make_converter();
    }
    if (data_converters.find("2") == data_converters.end())
    {
        data_converters["2"] = DataConverter::make_converter();
    }

    auto [yaw_raw, pitch_raw, roll_raw] = data_converters["1"]->rvec2Euler(raw_cam_pnp_data.rvec());
    auto [yaw_last, pitch_last, roll_last] = data_converters["2"]->rvec2Euler(last_raw_cam_pnp.rvec());
    float roll_correct = roll_raw;
    static constexpr float roll_limit = 36.0f / 180.0f * CV_PI;
    static constexpr float roll_limit_2 = 2 * roll_limit;

    while (roll_correct - roll_last > roll_limit)
    {
        roll_correct -= roll_limit_2;
    }
    while (roll_correct - roll_last < -roll_limit)
    {
        roll_correct += roll_limit_2;
    }
    roll_raw = roll_correct;

    // 将roll角转化为旋转向量
    if (data_converters.find("3") == data_converters.end())
    {
        data_converters["3"] = DataConverter::make_converter();
    }
    auto rvec_correct = data_converters["3"]->euler2Rvec(yaw_raw, pitch_raw, roll_raw);

    PoseNode raw_rune_to_cam_correct(rvec_correct, raw_cam_pnp_data.tvec());
    // 更新PNP数据
    corrected_cam_pnp_data = raw_rune_to_cam_correct;
    _correntRoll_last_cam_pnp_data = raw_rune_to_cam_correct;
    return true;
}
