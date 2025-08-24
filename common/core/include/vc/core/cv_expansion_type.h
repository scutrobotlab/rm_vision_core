/**
 * @file cv_expansion_type.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief OpenCV 常用矩阵类型拓展定义
 * @date 2025-7-15
 */

#pragma once

#include <opencv2/core.hpp>
#include "vc/core/type_expansion.hpp"

//----------------------【OpenCV 类型拓展】-----------------------
namespace cv
{
    /// @brief 5行1列浮点矩阵
    using Matx51f = cv::Matx<float, 5, 1>;

    /// @brief 5行2列浮点矩阵
    using Matx52f = cv::Matx<float, 5, 2>;

    /// @brief 5行3列浮点矩阵
    using Matx53f = cv::Matx<float, 5, 3>;

    /// @brief 5行4列浮点矩阵
    using Matx54f = cv::Matx<float, 5, 4>;

    /// @brief 5行5列浮点矩阵
    using Matx55f = cv::Matx<float, 5, 5>;
}
