#include "vc/feature/rune_filter_ekf.h"
#include "vc/feature/rune_filter_ekf_param.h"
#include <iostream>

using namespace cv;

RuneFilterEKF_CV::RuneFilterEKF_CV(RuneFilterDataType type) : _data_type(type)
{
    // 初始化状态向量
    x_ = Mat::zeros(12, 1, CV_64F);

    // 初始化协方差 P
    P_ = Mat::eye(12, 12, CV_64F) * rune_ekf_param.INIT_POS_VAR;
    for (int i = 6; i < 12; i++)
        P_.at<double>(i, i) = rune_ekf_param.INIT_VEL_VAR;

    // 过程噪声 Q
    Q_ = Mat::eye(12, 12, CV_64F);
    Q_(Rect(0, 0, 3, 3)) *= rune_ekf_param.Q_POS;
    Q_(Rect(3, 3, 3, 3)) *= rune_ekf_param.Q_ANGLE;
    Q_(Rect(6, 6, 3, 3)) *= rune_ekf_param.Q_VEL;
    Q_(Rect(9, 9, 3, 3)) *= rune_ekf_param.Q_ANGLE_VEL;

    // 观测噪声 R
    R_ = Mat::eye(6, 6, CV_64F);
    R_(Rect(0, 0, 3, 3)) *= rune_ekf_param.R_POS;
    R_(Rect(3, 3, 3, 3)) *= rune_ekf_param.R_ANGLE;

    // 观测矩阵 H (6x12)
    H_ = Mat::zeros(6, 12, CV_64F);
    for (int i = 0; i < 6; i++)
        H_.at<double>(i, i) = 1.0;
}

void RuneFilterEKF_CV::predict(double dt)
{
    Mat F = Mat::eye(12, 12, CV_64F);
    for (int i = 0; i < 3; i++)
    {
        F.at<double>(i, 6 + i) = dt;     // pos += vel * dt
        F.at<double>(3 + i, 9 + i) = dt; // angle += ang_vel * dt
    }
    x_ = F * x_;
    P_ = F * P_ * F.t() + Q_;
}

void RuneFilterEKF_CV::update(const Mat &z)
{
    Mat y = z - H_ * x_;                     // 创新
    Mat S = H_ * P_ * H_.t() + R_;           // 创新协方差
    Mat K = P_ * H_.t() * S.inv(DECOMP_SVD); // 卡尔曼增益
    x_ += K * y;
    Mat I = Mat::eye(12, 12, CV_64F);
    P_ = (I - K * H_) * P_;
}

RuneFilterStrategy::FilterOutput RuneFilterEKF_CV::filter(FilterInput input)
{
    if (!initialized_)
    {
        for (int i = 0; i < 6; i++)
            x_.at<double>(i, 0) = input.raw_pos(i);
        last_tick_ = input.tick;
        initialized_ = true;
        __latest_value = input.raw_pos;
        return {__latest_value};
    }

    dt_ = max((input.tick - last_tick_) / getTickFrequency(), rune_ekf_param.MIN_DT);
    last_tick_ = input.tick;

    predict(dt_);

    if (input.is_observation)
    {
        Mat z(6, 1, CV_64F);
        for (int i = 0; i < 6; i++)
            z.at<double>(i, 0) = input.raw_pos(i);
        update(z);
    }

    __filter_count++;
    for (int i = 0; i < 6; i++)
        __latest_value(i) = x_.at<double>(i, 0);

    return {__latest_value};
}

Matx61d RuneFilterEKF_CV::getPredict()
{
    Matx61d pred;
    for (int i = 0; i < 6; i++)
        pred(i) = x_.at<double>(i, 0);
    return pred;
}

bool RuneFilterEKF_CV::isValid()
{
    return initialized_;
}
