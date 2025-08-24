/**
 * @file extensions.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 适配 ContourWrapper 的 OpenCV 轮廓处理函数与绘制函数
 * @date 2025-04-12
 */

#pragma once

#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/core/logging.h"
#include <opencv2/core.hpp>
#include <iostream>
#include <unordered_map>

/**
 * @brief 增强版轮廓检测函数，返回智能轮廓对象集合
 *
 * @param image 输入二值图像 (建议使用 clone 保留原始数据)
 * @param contours 输出轮廓集合 (Contour_cptr)
 * @param hierarchy 输出轮廓层级信息
 * @param mode 轮廓检索模式 (默认为 cv::RETR_TREE)
 * @param method 轮廓近似方法 (默认为 cv::CHAIN_APPROX_NONE)
 * @param offset 轮廓点坐标偏移量 (默认为 (0,0))
 *
 * @note 自动执行抗锯齿处理 (根据 ENABLE_SMOOTH_CONTOUR_CALC 配置)
 */
inline void findContours(
    cv::InputArray image,
    std::vector<Contour_cptr> &contours,
    cv::OutputArray hierarchy,
    int mode = cv::RETR_TREE,
    int method = cv::CHAIN_APPROX_NONE,
    const cv::Point &offset = cv::Point(0, 0))
{
    std::vector<std::vector<cv::Point>> raw_contours;
    cv::findContours(image, raw_contours, hierarchy, mode, method, offset);
    contours.reserve(raw_contours.size());
    for (auto &&contour : raw_contours)
    {
        contours.emplace_back(ContourWrapper<int>::make_contour(std::move(contour)));
    }
}

/**
 * @brief 增强版轮廓检测函数，返回智能轮廓对象集合，使用 unordered_map 保存层级信息
 *
 * @param image 输入二值图像 (建议使用 clone 保留原始数据)
 * @param contours 输出轮廓集合 (Contour_cptr)
 * @param hierarchy 输出轮廓层级映射，格式为:
 *        std::unordered_map<当前轮廓, std::tuple<后一个轮廓, 前一个轮廓, 内嵌轮廓, 父轮廓>>
 * @param mode 轮廓检索模式 (默认为 cv::RETR_TREE)
 * @param method 轮廓近似方法 (默认为 cv::CHAIN_APPROX_NONE)
 * @param offset 轮廓点坐标偏移量 (默认为 (0,0))
 *
 * @note 自动执行抗锯齿处理 (根据 ENABLE_SMOOTH_CONTOUR_CALC 配置)
 */
inline void findContours(
    cv::InputArray image,
    std::vector<Contour_cptr> &contours,
    std::unordered_map<Contour_cptr, std::tuple<Contour_cptr, Contour_cptr, Contour_cptr, Contour_cptr>> &hierarchy,
    int mode = cv::RETR_TREE,
    int method = cv::CHAIN_APPROX_NONE,
    const cv::Point &offset = cv::Point(0, 0))
{
    std::vector<std::vector<cv::Point>> raw_contours;
    std::vector<cv::Vec4i> hierarchy_vec;
    cv::findContours(image, raw_contours, hierarchy_vec, mode, method, offset);
    contours.reserve(raw_contours.size());
    for (auto &&contour : raw_contours)
    {
        contours.emplace_back(ContourWrapper<int>::make_contour(std::move(contour)));
    }

    hierarchy.reserve(raw_contours.size());
    for (size_t i = 0; i < raw_contours.size(); ++i)
    {
        auto &contour = contours[i];
        auto &h = hierarchy_vec[i];
        hierarchy[contour] = std::make_tuple(
            (h[0] != -1) ? contours[h[0]] : nullptr,
            (h[1] != -1) ? contours[h[1]] : nullptr,
            (h[2] != -1) ? contours[h[2]] : nullptr,
            (h[3] != -1) ? contours[h[3]] : nullptr);
    }
}

/**
 * @brief 绘制单个 ContourWrapper 轮廓到图像
 *
 * @tparam T 轮廓数据类型 (int / float / double)
 * @param image 目标图像 (BGR)
 * @param contour 智能轮廓指针
 * @param color 绘制颜色 (默认绿色)
 * @param thickness 线宽 (默认 2)
 * @param lineType 线型 (默认抗锯齿)
 */
template <ContourWrapperBaseType T>
void drawContour(
    cv::Mat &image,
    const std::shared_ptr<const ContourWrapper<T>> &contour,
    const cv::Scalar &color = cv::Scalar(0, 255, 0),
    int thickness = 2,
    int lineType = cv::LINE_AA)
{
    if (!contour || contour->empty())
        return;

    const auto &points = contour->points();
    std::vector<cv::Point> int_points;
    int_points.reserve(points.size());

    if constexpr (std::is_same_v<T, int>)
    {
        std::transform(points.begin(), points.end(), std::back_inserter(int_points),
                       [](const cv::Point_<T> &p)
                       { return cv::Point(p.x, p.y); });
    }
    else
    {
        std::transform(points.begin(), points.end(), std::back_inserter(int_points),
                       [](const cv::Point_<T> &p)
                       { return cv::Point(cvRound(p.x), cvRound(p.y)); });
    }

    cv::drawContours(image, std::vector<std::vector<cv::Point>>{int_points}, 0, color, thickness, lineType);
}

/**
 * @brief 绘制轮廓集合到图像
 *
 * @param image 输入输出图像
 * @param contours 轮廓集合
 * @param contourIdx 绘制的轮廓索引 (-1 表示绘制所有)
 * @param color 绘制颜色
 * @param thickness 绘制线宽
 * @param lineType 绘制线型
 */
inline void drawContours(
    cv::InputOutputArray image,
    const std::vector<Contour_cptr> &contours,
    int contourIdx,
    const cv::Scalar &color,
    int thickness = 1,
    int lineType = cv::LINE_8)
{
    if (contourIdx < -1 || contourIdx >= static_cast<int>(contours.size()))
        throw std::out_of_range("Invalid contour index");

    for (size_t i = 0; i < contours.size(); ++i)
    {
        if (contourIdx == -1 || contourIdx == static_cast<int>(i))
        {
            const auto &contour = contours[i];
            if (contour)
                drawContour(image.getMatRef(), contour, color, thickness, lineType);
            else
                VC_WARNING_INFO("Contour at index %li is null.", i);
        }
    }
}

/**
 * @brief 删除指定索引轮廓，并更新层级信息
 *
 * @param contours 轮廓集合
 * @param hierarchy 层级向量
 * @param index 要删除的轮廓下标
 * @return true 删除成功, false 索引无效
 */
static bool deleteContour(
    std::vector<Contour_cptr> &contours,
    std::vector<cv::Vec4i> &hierarchy,
    int index)
{
    if (index < 0 || index >= contours.size())
        return false;

    auto updateHierarchy = [&](int idx, int new_val, int pos)
    {
        if (idx != -1)
            hierarchy[idx][pos] = new_val;
    };

    updateHierarchy(hierarchy[index][0], hierarchy[index][1], 1);
    updateHierarchy(hierarchy[index][1], hierarchy[index][0], 0);

    int sub_idx = hierarchy[index][2];
    while (sub_idx != -1)
    {
        hierarchy[sub_idx][3] = hierarchy[index][3];
        sub_idx = hierarchy[sub_idx][1];
    }

    if (hierarchy[hierarchy[index][3]][2] == index)
    {
        hierarchy[hierarchy[index][3]][2] = (hierarchy[index][2] != -1)   ? hierarchy[index][2]
                                            : (hierarchy[index][0] != -1) ? hierarchy[index][0]
                                            : (hierarchy[index][1] != -1) ? hierarchy[index][1]
                                                                          : -1;
    }

    for (auto &h : hierarchy)
        for (size_t i = 0; i < 4; ++i)
            if (h[i] > index)
                --h[i];

    contours.erase(contours.begin() + index);
    hierarchy.erase(hierarchy.begin() + index);

    return true;
}
