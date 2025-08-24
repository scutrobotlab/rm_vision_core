/**
 * @file geom_line.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 二维平面直线类定义与实现
 * @date 2025-7-15
 */

#pragma once

#include <opencv2/core.hpp>
#include <cmath>
#include <utility>
#include <limits>
#include <stdexcept>

/**
 * @brief 二维平面直线类模板
 *
 * 该类提供二维直线的构造、几何计算和分析功能，
 * 包括基于角度、两点生成直线、点到直线的距离计算、
 * 投影点、交点求解及一般式方程获取等。
 *
 * @tparam _Tp 数值类型，默认 double
 */
template <typename _Tp = double>
class Line2_
{
public:
    using value_type = _Tp;                    //!< 数值类型
    using point_type = cv::Point_<value_type>; //!< 二维点类型

    /**
     * @brief 默认构造函数
     * @note 默认生成水平直线：y = 0
     */
    Line2_();

    /**
     * @brief 通过角度和直线上一点构造直线
     *
     * @param[in] angle 直线角度（单位：弧度）
     * @param[in] point 直线上的一点
     * @note 当 angle = 0 时，直线方向水平，角度增加时直线逆时针旋转
     */
    Line2_(value_type angle, const point_type &point);

    /**
     * @brief 通过两点构造直线
     *
     * @param[in] p1 第一个点
     * @param[in] p2 第二个点
     * @throws std::invalid_argument 如果 p1 和 p2 重合
     */
    Line2_(const point_type &p1, const point_type &p2);

    /**
     * @brief 拷贝构造函数
     */
    Line2_(const Line2_ &other);

    /**
     * @brief 移动构造函数
     */
    Line2_(Line2_ &&other) noexcept;

    /**
     * @brief 拷贝赋值运算符
     */
    Line2_ &operator=(const Line2_ &other);

    /**
     * @brief 移动赋值运算符
     */
    Line2_ &operator=(Line2_ &&other) noexcept;

    /**
     * @brief 获取直线角度（弧度）
     * @return 直线角度，单位：弧度
     */
    value_type angle() const;

    /**
     * @brief 获取直线角度（角度制）
     * @return 直线角度，单位：度
     */
    value_type angleDegrees() const;

    /**
     * @brief 获取直线上的基准点
     * @return 基准点坐标
     */
    point_type point() const;

    /**
     * @brief 获取直线的方向向量
     * @return 单位方向向量
     */
    point_type direction() const;

    /**
     * @brief 获取直线的法向量
     * @return 法向量
     */
    point_type normal() const;

    /**
     * @brief 计算点到直线的距离
     * @param[in] p 任意二维点
     * @return 点到直线的距离
     */
    value_type distanceTo(const point_type &p) const;

    /**
     * @brief 判断点是否位于直线上
     *
     * @param[in] p 待检测的点
     * @param[in] tolerance 容差，默认 1e-5
     * @return true 点在直线上（容差范围内）
     */
    bool contains(const point_type &p, value_type tolerance = 1e-5) const;

    /**
     * @brief 计算两条直线的交点
     *
     * @param[in] other 另一条直线
     * @return 交点集合（无交点返回空）
     */
    std::vector<point_type> intersect(const Line2_ &other) const;

    /**
     * @brief 计算点在直线上的投影点
     *
     * @param[in] p 需要投影的点
     * @return 投影点坐标
     */
    point_type project(const point_type &p) const;

    /**
     * @brief 获取直线的一般式方程
     *
     * @param[out] A 一般式方程 Ax + By + C = 0 中的 A
     * @param[out] B 一般式方程中的 B
     * @param[out] C 一般式方程中的 C
     */
    void getGeneralEquation(value_type &A, value_type &B, value_type &C) const;

private:
    /**
     * @brief 规范化角度，使其范围在 [0, 2π)
     */
    void normalizeAngle();

    /**
     * @brief 根据参数 t 计算直线上对应点
     *
     * @param[in] t 参数
     * @return 直线上的点坐标
     */
    point_type pointAt(value_type t) const;

    value_type __angle;     //!< 直线角度（弧度）
    point_type __point;     //!< 直线上一点
    point_type __direction; //!< 单位方向向量
};

/** @typedef Line2i
 * @brief 使用 int 类型的二维直线
 */
using Line2i = Line2_<int>;

/** @typedef Line2f
 * @brief 使用 float 类型的二维直线
 */
using Line2f = Line2_<float>;

/** @typedef Line2d
 * @brief 使用 double 类型的二维直线
 */
using Line2d = Line2_<double>;

/** @typedef Line
 * @brief 默认二维直线类型（float）
 */
using Line = Line2f;

// 类外成员函数实现
template <typename _Tp>
Line2_<_Tp>::Line2_()
    : __angle(0), __point(0, 0), __direction(1, 0) {}

template <typename _Tp>
Line2_<_Tp>::Line2_(value_type angle, const point_type &point)
    : __angle(angle), __point(point),
      __direction(std::cos(angle), std::sin(angle))
{
    normalizeAngle();
}

template <typename _Tp>
Line2_<_Tp>::Line2_(const point_type &p1, const point_type &p2)
    : __point(p1)
{
    if (p1 == p2)
        throw std::invalid_argument("直线的两个点不能相同");

    cv::Point2d dir = p2 - p1;
    double len = cv::norm(dir);
    __direction = dir / len;
    __angle = std::atan2(__direction.y, __direction.x);

    normalizeAngle();
}

template <typename _Tp>
Line2_<_Tp>::Line2_(const Line2_ &other)
    : __angle(other.__angle), __point(other.__point), __direction(other.__direction) {}

template <typename _Tp>
Line2_<_Tp>::Line2_(Line2_ &&other) noexcept
    : __angle(std::move(other.__angle)), __point(std::move(other.__point)),
      __direction(std::move(other.__direction)) {}

template <typename _Tp>
Line2_<_Tp> &Line2_<_Tp>::operator=(const Line2_ &other)
{
    if (this != &other)
    {
        __angle = other.__angle;
        __point = other.__point;
        __direction = other.__direction;
    }
    return *this;
}

template <typename _Tp>
Line2_<_Tp> &Line2_<_Tp>::operator=(Line2_ &&other) noexcept
{
    if (this != &other)
    {
        __angle = std::move(other.__angle);
        __point = std::move(other.__point);
        __direction = std::move(other.__direction);
    }
    return *this;
}

template <typename _Tp>
void Line2_<_Tp>::normalizeAngle()
{
    constexpr value_type twoPi = 2 * static_cast<value_type>(CV_PI);
    __angle = std::fmod(__angle, twoPi);
    if (__angle < 0)
        __angle += twoPi;
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::pointAt(value_type t) const
{
    return __point + t * __direction;
}

template <typename _Tp>
typename Line2_<_Tp>::value_type Line2_<_Tp>::angle() const
{
    return __angle;
}

template <typename _Tp>
typename Line2_<_Tp>::value_type Line2_<_Tp>::angleDegrees() const
{
    return __angle * 180.0 / CV_PI;
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::point() const
{
    return __point;
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::direction() const
{
    return __direction;
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::normal() const
{
    return point_type(-__direction.y, __direction.x);
}

template <typename _Tp>
typename Line2_<_Tp>::value_type Line2_<_Tp>::distanceTo(const point_type &p) const
{
    cv::Point2d diff = p - __point;
    return std::abs(diff.dot(normal()));
}

template <typename _Tp>
bool Line2_<_Tp>::contains(const point_type &p, value_type tolerance) const
{
    return std::abs(distanceTo(p)) < tolerance;
}

template <typename _Tp>
std::vector<typename Line2_<_Tp>::point_type> Line2_<_Tp>::intersect(const Line2_ &other) const
{
    const double tolerance = std::numeric_limits<value_type>::epsilon();
    const double det = __direction.x * other.__direction.y -
                       __direction.y * other.__direction.x;

    if (std::abs(det) < tolerance)
        return {};

    cv::Point2d diff = other.__point - __point;
    const double t = (diff.x * other.__direction.y - diff.y * other.__direction.x) / det;
    return {pointAt(t)};
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::project(const point_type &p) const
{
    // 计算点到直线上的向量
    cv::Point2d vec = p - __point;
    value_type t = vec.dot(__direction);
    return __point + t * __direction;
}

template <typename _Tp>
void Line2_<_Tp>::getGeneralEquation(value_type &A, value_type &B, value_type &C) const
{
    A = -__direction.y;
    B = __direction.x;
    C = __direction.y * __point.x - __direction.x * __point.y;
}
