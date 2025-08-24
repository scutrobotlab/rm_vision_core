/**
 * @file geom_utils.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 常用几何与向量计算工具头文件
 * @date 2025-7-15
 */

#pragma once

#include <opencv2/core.hpp>

#include "vc/math/type_utils.hpp"
#include "vc/core/logging.h"

//! 角度制
enum AngleMode : bool
{
    RAD = true,
    DEG = false
};

//! 欧拉角转轴枚举
enum EulerAxis : int
{
    X = 0, //!< X 轴
    Y = 1, //!< Y 轴
    Z = 2  //!< Z 轴
};

// -------------------- 角度规范化函数 (取代宏) --------------------
template <typename T>
inline T NormalizeDegree(T degrees)
{
    while (degrees > 180)
        degrees -= 360;
    while (degrees <= -180)
        degrees += 360;
    return degrees;
}

template <typename T>
inline T NormalizeRadian(T radians)
{
    while (radians > CV_PI)
        radians -= 2 * CV_PI;
    while (radians <= -CV_PI)
        radians += 2 * CV_PI;
    return radians;
}

// ------------------------【常用变换公式】------------------------

/**
 * @brief 角度转换为弧度
 *
 * @tparam Tp 变量类型
 * @param[in] deg 角度
 * @return 弧度
 */
template <typename Tp>
inline Tp deg2rad(Tp deg)
{
    return deg * static_cast<Tp>(CV_PI) / static_cast<Tp>(180);
}

/**
 * @brief 弧度转换为角度
 *
 * @tparam Tp 变量类型
 * @param[in] rad 弧度
 * @return 角度
 */
template <typename Tp>
inline Tp rad2deg(Tp rad) { return rad * static_cast<Tp>(180) / static_cast<Tp>(CV_PI); }

/**
 * @brief Point类型转换为Matx类型
 *
 * @tparam Tp 数据类型
 * @param[in] point Point类型变量
 * @return Matx类型变量
 */
template <typename Tp>
inline cv::Matx<Tp, 3, 1> point2matx(cv::Point3_<Tp> point) { return cv::Matx<Tp, 3, 1>(point.x, point.y, point.z); }

/**
 * @brief Matx类型转换为Point类型
 *
 * @tparam Tp 数据类型
 * @param[in] matx Matx类型变量
 * @return Point类型变量
 */
template <typename Tp>
inline cv::Point3_<Tp> matx2point(cv::Matx<Tp, 3, 1> matx) { return cv::Point3_<Tp>(matx(0), matx(1), matx(2)); }

/**
 * @brief Matx类型转换为Vec类型
 *
 * @tparam Tp 数据类型
 * @param[in] matx Matx类型变量
 * @return Vec类型变量
 */
template <typename Tp>
inline cv::Vec<Tp, 3> matx2vec(cv::Matx<Tp, 3, 1> matx) { return cv::Vec<Tp, 3>(matx(0), matx(1), matx(2)); }

// ------------------------【几何距离计算】------------------------

/**
 * @brief 计算二维向量之间的欧氏距离
 *
 * @param v1 第一个向量
 * @param v2 第二个向量
 * @return 两个向量之间的距离
 */
template <geom_utils_concepts::vector_2_type Vec1,
          geom_utils_concepts::vector_2_type Vec2>
inline auto getDist(const Vec1 &v1, const Vec2 &v2) noexcept
{
    using namespace geom_utils_concepts;
    const auto dx = get_x(v1) - get_x(v2);
    const auto dy = get_y(v1) - get_y(v2);
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 计算三维向量之间的欧氏距离
 *
 * @param v1 第一个三维向量
 * @param v2 第二个三维向量
 * @return 两个三维向量之间的距离
 */
template <geom_utils_concepts::vector_3_type Vec1,
          geom_utils_concepts::vector_3_type Vec2>
inline auto getDist(const Vec1 &v1, const Vec2 &v2) noexcept
{
    using namespace geom_utils_concepts;
    const auto dx = get_x(v1) - get_x(v2);
    const auto dy = get_y(v1) - get_y(v2);
    const auto dz = get_z(v1) - get_z(v2);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 计算单位向量
 *
 * @tparam Tp 数据类型
 * @param[in] v 向量
 * @return 向量在单位圆上的向量
 */
template <typename Tp>
inline cv::Point_<Tp> getUnitVector(const cv::Point_<Tp> &v)
{
    return v / sqrt(v.x * v.x + v.y * v.y);
}

/**
 * @brief 求两个向量的最小夹角
 *
 * @tparam Tp 数据类型
 * @param[in] v1 向量1
 * @param[in] v2 向量2
 * @param[in] mode 角度模式，默认弧度制
 * @return 夹角大小,[0,180]
 */
template <typename Tp1, typename Tp2>
inline Tp1 getVectorMinAngle(const cv::Point_<Tp1> &v1, const cv::Point_<Tp2> &v2, AngleMode mode = RAD)
{
    Tp1 cos_theta = (v1.x * v2.x + v1.y * v2.y) / (std::sqrt(v1.x * v1.x + v1.y * v1.y) * std::sqrt(v2.x * v2.x + v2.y * v2.y));
    Tp1 theta = std::acos(cos_theta);
    if (std::isnan(theta))
        return Tp1(0);
    return mode ? theta : rad2deg(theta);
}

/**
 * @brief 计算点集的方向向量 (逐差法)
 *
 * @tparam Tp 数据类型
 * @param[in] points 点集
 * @return 点集的整体方向，以 points[0] 指向 points[1] 为正方向
 */
template <typename Tp>
inline cv::Point2f getPointsDirectionVector(const std::vector<cv::Point_<Tp>> &points)
{
    cv::Point2f direction{};
    if (points.size() <= 1)
    {
        return direction;
    }

    size_t half_size = points.size() / 2;
    for (size_t i = 0; i < half_size; ++i)
    {
        direction -= static_cast<cv::Point2f>(points[i]);
    }
    for (size_t i = half_size + (points.size() % 2); i < points.size(); ++i)
    {
        direction += static_cast<cv::Point2f>(points[i]);
    }

    direction /= static_cast<float>(half_size * (half_size + (points.size() % 2)));
    float norm = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (norm > std::numeric_limits<float>::epsilon())
    {
        direction /= norm;
    }

    return direction;
}

/**
 * @brief 计算两直线交点
 *
 * @tparam Tp 数据类型
 * @param[in] p1 直线1上的点1
 * @param[in] p2 直线1上的点2
 * @param[in] p3 直线2上的点1
 * @param[in] p4 直线2上的点2
 * @return 交点坐标
 * @note 若两直线平行，返回 (std::numeric_limits<Tp>::quiet_NaN(), std::numeric_limits<Tp>::quiet_NaN())
 */
template <typename Tp>
inline cv::Point_<Tp> getLineIntersection(const cv::Point_<Tp> &p1, const cv::Point_<Tp> &p2, const cv::Point_<Tp> &p3, const cv::Point_<Tp> &p4)
{
    Tp A1 = p2.y - p1.y;
    Tp B1 = p1.x - p2.x;
    Tp C1 = A1 * p1.x + B1 * p1.y;

    Tp A2 = p4.y - p3.y;
    Tp B2 = p3.x - p4.x;
    Tp C2 = A2 * p3.x + B2 * p3.y;

    Tp delta = A1 * B2 - A2 * B1;

    if (std::abs(delta) < std::numeric_limits<double>::epsilon())
    {
        return cv::Point_<Tp>(std::numeric_limits<Tp>::quiet_NaN(), std::numeric_limits<Tp>::quiet_NaN());
    }
    Tp x = static_cast<int>((B2 * C1 - B1 * C2) / delta);
    Tp y = static_cast<int>((A1 * C2 - A2 * C1) / delta);
    return cv::Point_<Tp>(x, y);
}

/**
 * @brief 计算两直线交点
 *
 * @tparam Tp 数据类型
 * @param[in] line1 直线1 (vx, vy, x0, y0)
 * @param[in] line2 直线2  (vx, vy, x0, y0)
 * @return 交点坐标
 * @note 若两直线平行，返回 (std::numeric_limits<Tp>::quiet_NaN(), std::numeric_limits<Tp>::quiet_NaN())
 */
template <typename Tp1, typename Tp2>
inline auto getLineIntersection(const cv::Vec<Tp1, 4> &line1, const cv::Vec<Tp2, 4> &line2)
{
    auto A1 = line1[1], B1 = -line1[0], C1 = line1[0] * line1[3] - line1[1] * line1[2];
    auto A2 = line2[1], B2 = -line2[0], C2 = line2[0] * line2[3] - line2[1] * line2[2];
    if (auto delta = A1 * B2 - A2 * B1; std::abs(delta) < std::numeric_limits<decltype(delta)>::epsilon())
    {
        using ResultT = decltype((B1 * C2 - B2 * C1) / delta);
        return cv::Point_<ResultT>(std::numeric_limits<ResultT>::quiet_NaN(),
                                   std::numeric_limits<ResultT>::quiet_NaN());
    }
    else
    {
        return cv::Point_((B1 * C2 - B2 * C1) / delta, (A2 * C1 - A1 * C2) / delta);
    }
}

/**
 * @brief 计算投影向量
 *
 * @tparam Tp 数据类型
 * @param[in] v1 向量1
 * @param[in] v2 向量2
 * @return 向量1在向量2上的投影向量
 */
template <typename Tp>
inline cv::Point_<Tp> getProjectionVector(const cv::Point_<Tp> &v1, const cv::Point_<Tp> &v2)
{
    return (v1.x * v2.x + v1.y * v2.y) / (v2.x * v2.x + v2.y * v2.y) * v2;
}

/**
 * @brief 计算投影长度
 *
 * @tparam Tp 数据类型
 * @param[in] v1 向量1
 * @param[in] v2 向量2
 * @return 向量1在向量2上的投影长度
 */
template <typename Tp>
inline Tp getProjection(const cv::Point_<Tp> &v1, const cv::Point_<Tp> &v2)
{
    return (v1.x * v2.x + v1.y * v2.y) / sqrt(v2.x * v2.x + v2.y * v2.y);
}

/**
 * @brief 平面向量外积计算
 *
 * @param v1 第一个向量
 * @param v2 第二个向量
 * @return 外积结果 , 若 retval = 0,则共线
 */
template <geom_utils_concepts::vector_2_type Vec1,
          geom_utils_concepts::vector_2_type Vec2>
inline auto getCross(const Vec1 &v1, const Vec2 &v2) noexcept
{
    using namespace geom_utils_concepts;
    const auto v1_x = get_x(v1);
    const auto v1_y = get_y(v1);
    const auto v2_x = get_x(v2);
    const auto v2_y = get_y(v2);
    return v1_x * v2_y - v1_y * v2_x; // 返回外积结果
}

/**
 * @brief 欧拉角转换为旋转矩阵
 *
 * @tparam _Tp 数据类型
 * @param[in] val 角度数值（弧度制）
 * @param[in] axis 转轴
 * @return Matx 格式的旋转矩阵
 */
template <typename _Tp>
inline cv::Matx<_Tp, 3, 3> euler2Mat(_Tp val, EulerAxis axis)
{
    _Tp s = std::sin(val), c = std::cos(val);
    switch (axis)
    {
    case X:
        return {1, 0, 0, 0, c, -s, 0, s, c};
    case Y:
        return {c, 0, s, 0, 1, 0, -s, 0, c};
    case Z:
        return {c, -s, 0, s, c, 0, 0, 0, 1};
    default:
        VC_THROW_ERROR("Bad argument of the \"axis\": %d", static_cast<int>(axis));
        return cv::Matx<_Tp, 3, 3>::eye();
    }
}

/**
 * @brief 获取两条直线的交点
 *
 * @param[in] LineA 直线1(两点式表示法)
 * @param[in] LineB 直线2（两点式表示法）
 */
template <typename Tp1, typename Tp2>
inline cv::Point2f getCrossPoint(const cv::Vec<Tp1, 4> &LineA, const cv::Vec<Tp2, 4> &LineB)
{
    double ka, kb;
    ka = static_cast<double>(LineA[3] - LineA[1]) / static_cast<double>(LineA[2] - LineA[0]);
    kb = static_cast<double>(LineB[3] - LineB[1]) / static_cast<double>(LineB[2] - LineB[0]);
    cv::Point2f crossPoint;
    crossPoint.x = static_cast<float>((ka * LineA[0] - LineA[1] - kb * LineB[0] + LineB[1]) / (ka - kb));
    crossPoint.y = static_cast<float>((ka * kb * (LineA[0] - LineB[0]) + ka * LineB[1] - kb * LineA[1]) / (ka - kb));
    return crossPoint;
}

/**
 * @brief 点到直线距离
 * @note 点 \f$P=(x_0,y_0)\f$ 到直线 \f$l:Ax+By+C=0\f$ 距离公式为
 *       \f[D(P,l)=\frac{Ax_0+By_0+C}{\sqrt{A^2+B^2}}\f]
 *
 * @tparam Tp1 直线方程数据类型
 * @tparam Tp2 平面点的数据类型
 * @param[in] line 用 `cv::Vec4_` 表示的直线方程 (vx, vy, x0, y0)
 * @param[in] pt 平面点
 * @param[in] direc 是否区分距离的方向
 * @note
 * - 计算结果有正负，即区分点在直线分布的方向，通过修改传入参数
 *   `direc = false` 来设置计算结果不区分正负（统一返回 \f$|D(P,l)|\f$）
 * - `line` 需要传入元素为 \f$(v_x, v_y, x_0, y_0)\f$ 的用 `cv::Vec4_`
 *   表示的向量，指代下列直线方程（与 `cv::fitLine` 传入的 `cv::Vec4_`
 *   参数一致）\f[l:y-y_0=\frac{v_y}{v_x}(x-x_0)\f]
 * @return 平面欧式距离
 */
template <typename Tp1, typename Tp2>
inline auto getDist(const cv::Vec<Tp1, 4> &line, const cv::Point_<Tp2> &pt, bool direc = true)
{
    auto retval = (line(1) * pt.x - line(0) * pt.y +
                   line(0) * line(3) - line(1) * line(2)) /
                  std::sqrt(line(0) * line(0) + line(1) * line(1));
    return direc ? retval : std::abs(retval);
}

/**
 * @brief 将陀螺仪欧拉角转化为旋转矩阵
 *
 * @param[in] yaw 陀螺仪 yaw 数据 (左负右正)
 * @param[in] pitch 陀螺仪 pitch 数据 (上负下正)
 */
template <typename _Tp>
inline cv::Matx<_Tp, 3, 3> gyroEuler2RotMat(_Tp yaw, _Tp pitch)
{
    return euler2Mat(deg2rad(yaw), Y) * euler2Mat(deg2rad(-1 * pitch), X);
}
