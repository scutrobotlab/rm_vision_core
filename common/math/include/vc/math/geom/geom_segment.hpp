/**
 * @file geom_segment.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 二维线段类头文件
 * @date 2025-7-15
 *
 * @details
 * 该类用于表示二维空间中的线段，并提供如下几何操作：
 * - 计算线段长度
 * - 判断点是否在线段上
 * - 计算点到线段的距离
 * - 点投影和最近点计算
 * - 判断两条线段是否相交以及求交点
 */

#pragma once

#include <opencv2/core.hpp>
#include <cmath>
#include <utility>
#include <limits>
#include <stdexcept>

/**
 * @brief 二维线段类模板
 * @tparam _Tp 线段数值类型，默认 double
 */
template <typename _Tp = double>
class Segment2_
{
public:
    using value_type = _Tp;                    //!< 线段的数值类型
    using point_type = cv::Point_<value_type>; //!< 点类型

    /**
     * @brief 默认构造函数，创建单位线段
     * @note 起点在原点 (0,0)，终点在 (1,0)
     */
    Segment2_();

    /**
     * @brief 构造函数，通过两个点创建线段
     * @param[in] pt1 线段的起点
     * @param[in] pt2 线段的终点
     * @note 如果两点相同，线段长度为零
     */
    Segment2_(const point_type &pt1, const point_type &pt2);

    /**
     * @brief 拷贝构造函数
     * @param[in] other 另一条线段
     */
    Segment2_(const Segment2_ &other);

    /**
     * @brief 移动构造函数
     * @param[in] other 另一条线段（右值引用）
     */
    Segment2_(Segment2_ &&other) noexcept;

    /**
     * @brief 赋值运算符重载
     * @param[in] other 另一条线段
     * @return 当前线段对象的引用
     */
    Segment2_ &operator=(const Segment2_ &other);

    /**
     * @brief 移动赋值运算符重载
     * @param[in] other 另一条线段（右值引用）
     * @return 当前线段对象的引用
     */
    Segment2_ &operator=(Segment2_ &&other) noexcept;

    /**
     * @brief 获取线段的起点
     * @return 起点坐标
     */
    point_type start() const;

    /**
     * @brief 获取线段的终点
     * @return 终点坐标
     */
    point_type end() const;

    /**
     * @brief 获取线段的方向向量（终点减起点）
     * @return 线段向量
     */
    point_type vector() const;

    /**
     * @brief 计算线段长度
     * @return 线段的长度
     */
    value_type length() const;

    /**
     * @brief 计算点在线段上的投影点
     * @param[in] p 外部点
     * @return 投影点坐标（可能在延长线上）
     */
    point_type project(const point_type &p) const;

    /**
     * @brief 计算点到线段的最近点
     * @param[in] p 外部点
     * @return 最近点坐标（一定在线段上）
     */
    point_type closestPoint(const point_type &p) const;

    /**
     * @brief 计算点到线段的距离
     * @param[in] p 外部点
     * @return 最近距离
     */
    value_type distanceTo(const point_type &p) const;

    /**
     * @brief 检查点是否在线段上
     * @param[in] p 要检查的点
     * @param[in] tolerance 容差值，默认1e-5
     * @return true 点在线段上；false 不在线段上
     */
    bool contains(const point_type &p, value_type tolerance = 1e-5) const;

    /**
     * @brief 判断两条线段是否相交
     * @param[in] other 另一条线段
     * @return true 相交或端点接触；false 不相交
     */
    bool intersects(const Segment2_ &other) const;

    /**
     * @brief 计算两条线段的交点
     * @param[in] other 另一条线段
     * @return 交点集合（0或1个点，共线重叠时返回多个端点）
     */
    std::vector<point_type> intersect(const Segment2_ &other) const;

private:
    point_type __pt1; //!< 线段起点
    point_type __pt2; //!< 线段终点
};

// ================== 类外函数实现 ==================

template <typename _Tp>
Segment2_<_Tp>::Segment2_() : __pt1(0, 0), __pt2(1, 0) {}

template <typename _Tp>
Segment2_<_Tp>::Segment2_(const point_type &pt1, const point_type &pt2) : __pt1(pt1), __pt2(pt2) {}

template <typename _Tp>
Segment2_<_Tp>::Segment2_(const Segment2_ &other) : __pt1(other.__pt1), __pt2(other.__pt2) {}

template <typename _Tp>
Segment2_<_Tp>::Segment2_(Segment2_ &&other) noexcept : __pt1(std::move(other.__pt1)), __pt2(std::move(other.__pt2)) {}

template <typename _Tp>
Segment2_<_Tp> &Segment2_<_Tp>::operator=(const Segment2_ &other)
{
    if (this != &other)
    {
        __pt1 = other.__pt1;
        __pt2 = other.__pt2;
    }
    return *this;
}

template <typename _Tp>
Segment2_<_Tp> &Segment2_<_Tp>::operator=(Segment2_ &&other) noexcept
{
    if (this != &other)
    {
        __pt1 = std::move(other.__pt1);
        __pt2 = std::move(other.__pt2);
    }
    return *this;
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::start() const
{
    return __pt1;
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::end() const
{
    return __pt2;
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::vector() const
{
    return __pt2 - __pt1;
}

template <typename _Tp>
typename Segment2_<_Tp>::value_type Segment2_<_Tp>::length() const
{
    return std::sqrt((__pt2.x - __pt1.x) * (__pt2.x - __pt1.x) + (__pt2.y - __pt1.y) * (__pt2.y - __pt1.y));
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::project(const point_type &p) const
{
    point_type vec = vector();
    point_type diff = p - __pt1;
    value_type len_sq = vec.x * vec.x + vec.y * vec.y;
    if (len_sq < std::numeric_limits<value_type>::epsilon())
    {
        return __pt1;
    }
    value_type t = diff.dot(vec) / len_sq;
    return __pt1 + t * vec;
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::closestPoint(const point_type &p) const
{
    point_type proj = project(p);
    point_type vec = vector();
    value_type t;
    if (std::abs(vec.x) > std::abs(vec.y))
    {
        t = (proj.x - __pt1.x) / vec.x;
    }
    else if (vec.y != 0)
    {
        t = (proj.y - __pt1.y) / vec.y;
    }
    else
    {
        t = 0;
    }
    if (t < 0)
        return __pt1;
    if (t > 1)
        return __pt2;
    return proj;
}

template <typename _Tp>
typename Segment2_<_Tp>::value_type Segment2_<_Tp>::distanceTo(const point_type &p) const
{
    point_type closest = closestPoint(p);
    value_type dx = p.x - closest.x;
    value_type dy = p.y - closest.y;
    return std::sqrt(dx * dx + dy * dy);
}

template <typename _Tp>
bool Segment2_<_Tp>::contains(const point_type &p, value_type tolerance) const
{
    point_type proj = project(p);
    value_type dx = p.x - proj.x;
    value_type dy = p.y - proj.y;
    if (dx * dx + dy * dy > tolerance * tolerance)
    {
        return false;
    }
    point_type closest = closestPoint(p);
    if (closest == __pt1 || closest == __pt2)
    {
        return true;
    }
    point_type v1 = closest - __pt1;
    point_type v2 = closest - __pt2;
    return v1.dot(v2) < tolerance;
}

template <typename _Tp>
bool Segment2_<_Tp>::intersects(const Segment2_ &other) const
{
    point_type p1 = __pt1;
    point_type q1 = __pt2;
    point_type p2 = other.__pt1;
    point_type q2 = other.__pt2;
    point_type r = q1 - p1;
    point_type s = q2 - p2;
    point_type pq = p2 - p1;
    value_type rxs = r.cross(s);
    value_type pqxs = pq.cross(s);
    value_type pqxr = pq.cross(r);
    const value_type tolerance = std::numeric_limits<value_type>::epsilon();
    if (std::abs(rxs) < tolerance)
    {
        if (std::abs(pqxr) > tolerance)
        {
            return false;
        }
        value_type t0 = pq.dot(r) / r.dot(r);
        value_type t1 = t0 + s.dot(r) / r.dot(r);
        if (s.dot(r) < 0)
        {
            std::swap(t0, t1);
        }
        return !(t1 < 0 || t0 > 1);
    }
    value_type t = pqxs / rxs;
    value_type u = pqxr / rxs;
    return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
}

template <typename _Tp>
std::vector<typename Segment2_<_Tp>::point_type> Segment2_<_Tp>::intersect(const Segment2_ &other) const
{
    point_type p1 = __pt1;
    point_type q1 = __pt2;
    point_type p2 = other.__pt1;
    point_type q2 = other.__pt2;
    point_type r = q1 - p1;
    point_type s = q2 - p2;
    point_type pq = p2 - p1;
    value_type rxs = r.cross(s);
    value_type pqxs = pq.cross(s);
    value_type pqxr = pq.cross(r);
    const value_type tolerance = std::numeric_limits<value_type>::epsilon();
    std::vector<point_type> intersections;
    if (std::abs(rxs) < tolerance && std::abs(pqxr) < tolerance)
    {
        value_type t0 = pq.dot(r) / r.dot(r);
        value_type t1 = t0 + s.dot(r) / r.dot(r);
        value_type start_t = std::max<value_type>(0, std::min(t0, t1));
        value_type end_t = std::min<value_type>(1, std::max(t0, t1));
        if (start_t <= end_t)
        {
            point_type start_pt = p1 + start_t * r;
            point_type end_pt = p1 + end_t * r;
            intersections.push_back(start_pt);
            if (std::abs(end_t - start_t) > tolerance)
            {
                intersections.push_back(end_pt);
            }
        }
        return intersections;
    }
    if (std::abs(rxs) > tolerance)
    {
        value_type t = pqxs / rxs;
        value_type u = pqxr / rxs;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        {
            intersections.push_back(p1 + t * r);
        }
    }
    return intersections;
}

using Segment2i = Segment2_<int>;    //!< 整型线段类
using Segment2f = Segment2_<float>;  //!< 单精度浮点数线段类
using Segment2d = Segment2_<double>; //!< 双精度浮点数线段类
using Segment = Segment2f;           //!< 默认线段类型，单精度浮点数
