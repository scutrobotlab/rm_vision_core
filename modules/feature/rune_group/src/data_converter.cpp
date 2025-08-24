#include "vc/feature/rune_group.h"
#include <tuple>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

std::shared_ptr<DataConverter> DataConverter::make_converter()
{
    return std::make_shared<DataConverter>();
}

std::tuple<Matx31f, Matx31f> DataConverter::toTvecAndRvec(const Matx61f &filter_pos)
{
    auto rvec = euler2Rvec(deg2rad(filter_pos(3)), deg2rad(filter_pos(4)), deg2rad(filter_pos(5)));
    return {Matx31f{filter_pos(0), filter_pos(1), filter_pos(2)}, rvec};
}

Matx61f DataConverter::toFilterForm(const Vec3f &tvec, const Vec3f &rvec, bool &is_gimbal_lock, bool is_reset)
{
    auto [yaw, pitch, roll] = rvec2Euler(rvec);

    if (is_reset)
    {
        _last_yaw = yaw;
        _last_roll = roll;
    }
    else
    {
        while (yaw - _last_yaw > CV_PI)
            yaw -= 2 * CV_PI;
        while (yaw - _last_yaw < -CV_PI)
            yaw += 2 * CV_PI;
        _last_yaw = yaw;

        while (roll - _last_roll > CV_PI)
            roll -= 2 * CV_PI;
        while (roll - _last_roll < -CV_PI)
            roll += 2 * CV_PI;
        _last_roll = roll;
    }

    pitch = rad2deg(pitch);
    yaw = rad2deg(yaw);
    roll = rad2deg(roll);
    is_gimbal_lock = (pitch > 85.0f || pitch < -85.0f);
    return Matx61f{tvec(0), tvec(1), tvec(2), yaw, pitch, roll};
}

Matx31f DataConverter::euler2Rvec(float yaw, float pitch, float roll)
{
    Matx33f rmat = euler2Mat(yaw, EulerAxis::Y) * euler2Mat(pitch, EulerAxis::X) * euler2Mat(roll, EulerAxis::Z);
    Matx31f rvec;
    Rodrigues(rmat, rvec);
    return rvec;
}

std::tuple<float, float, float> DataConverter::rvec2Euler(const Vec3f &rvec)
{
    Matx33f rmat;
    Rodrigues(rvec, rmat);
    float pitch = asin(clamp(-rmat(1, 2), -1.0f, 1.0f));
    float yaw = atan2(rmat(0, 2), rmat(2, 2));
    float roll = atan2(rmat(1, 0), rmat(1, 1));
    return {yaw, pitch, roll};
}

Matx61f DataConverter::cvtPos(const Vec3f &tvec, const Vec3f &rvec, bool &is_gimbal_lock)
{
    auto [yaw, pitch, roll] = rvec2Euler(rvec);
    yaw = rad2deg(yaw);
    pitch = rad2deg(pitch);
    roll = rad2deg(roll);
    is_gimbal_lock = (pitch > 85.0f || pitch < -85.0f);
    return Matx61f{tvec(0), tvec(1), tvec(2), yaw, pitch, roll};
}
