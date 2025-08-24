/**
 * @file hierarchy.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 轮廓的层级结构处理函数
 * @date 2025-01-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <vector>
#include <unordered_set>
#include <opencv2/opencv.hpp>

#include "vc/contour_proc/contour_wrapper.hpp"

/**
 * @brief 递归获取指定轮廓的所有子轮廓下标（实现函数）
 *
 * @tparam T 容器类型，需要支持 insert 接口（如 std::vector<int>）
 * @param hierarchy 所有轮廓的层级结构，由 OpenCV findContours 返回
 * @param idx 当前轮廓下标
 * @param sub_contours_idx 用于存储结果的容器
 *
 * @note 遍历当前轮廓的子节点，并递归获取所有子轮廓的索引。
 */
template <typename T>
void getAllSubContoursIdxImpl(const std::vector<cv::Vec4i> &hierarchy, int idx, T &sub_contours_idx)
{
    using namespace std;
    using namespace cv;

    if (hierarchy[idx][2] == -1)
        return;
    int front_child_idx = hierarchy[idx][2];
    while (front_child_idx != -1)
    {
        sub_contours_idx.insert(sub_contours_idx.end(), static_cast<typename T::value_type>(front_child_idx)); //!< 插入子轮廓下标
        getAllSubContoursIdxImpl(hierarchy, front_child_idx, sub_contours_idx);                                //!< 递归获取子轮廓
        front_child_idx = hierarchy[front_child_idx][1];                                                       //!< 获取下一个兄弟轮廓下标
    }
    if (hierarchy[hierarchy[idx][2]][0] == -1)
        return;
    int back_child_idx = hierarchy[hierarchy[idx][2]][0];
    while (back_child_idx != -1)
    {
        sub_contours_idx.insert(sub_contours_idx.end(), static_cast<typename T::value_type>(back_child_idx)); //!< 插入子轮廓下标
        getAllSubContoursIdxImpl(hierarchy, back_child_idx, sub_contours_idx);                                //!< 递归获取子轮廓
        back_child_idx = hierarchy[back_child_idx][0];                                                        //!< 获取下一个兄弟轮廓下标
    }
}

/**
 * @brief 获取指定轮廓的所有子轮廓下标（接口函数）
 *
 * @tparam T 容器类型，需要支持 insert 接口（如 std::vector<int>）
 * @param hierarchy 所有轮廓的层级结构，由 OpenCV findContours 返回
 * @param idx 指定轮廓下标
 * @param sub_contours_idx 用于存储结果的容器
 *
 * @note 调用内部递归实现函数完成子轮廓索引的收集。
 */
template <typename T>
void getAllSubContoursIdx(const std::vector<cv::Vec4i> &hierarchy, int idx, T &sub_contours_idx)
{
    getAllSubContoursIdxImpl<T>(hierarchy, idx, sub_contours_idx);
}
