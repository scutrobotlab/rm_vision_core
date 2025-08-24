/**
 * @file transform6D.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 6自由度变换类 Transform6D 定义，支持旋转矩阵、旋转向量和平移向量的转换及组合操作
 * @date 2025-08-24
 */

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>

#include "vc/math/type_utils.hpp"
#include "vc/math/geom_utils.hpp"

//! Transform6D 的传入参数类型概念
namespace transform6D_concepts
{
    /**
     * @brief 去除常量和引用限定的旋转矩阵类型概念
     */
    template <typename T>
    using base_type = std::remove_cvref_t<T>;

    /**
     * @brief 旋转矩阵类型概念
     * @tparam T 旋转矩阵类型
     *
     * @note 要求 T 为 cv::Matx<Tp, 3, 3>
     */
    template <typename T>
    concept rmat_type = requires(T a) {
        requires std::is_same_v<base_type<T>, cv::Matx<typename T::value_type, 3, 3>>;
        requires geom_utils_concepts::vector_arithmetic<typename T::value_type>;
    };

    /**
     * @brief 旋转向量类型概念
     * @tparam T 旋转向量类型
     */
    template <typename T>
    concept rvec_type = geom_utils_concepts::vector_3_type<T>;

    /**
     * @brief 平移向量类型概念
     * @tparam T 平移向量类型
     */
    template <typename T>
    concept tvec_type = geom_utils_concepts::vector_3_type<T>;
}

class Transform6D
{
public:
    //! 旋转矩阵存储类型
    using RmatType = cv::Matx33d;
    //! 旋转向量存储类型
    using RvecType = cv::Vec3d;
    //! 平移向量存储类型
    using TvecType = cv::Vec3d;

    /**
     * @brief 默认构造函数
     */
    Transform6D();

    Transform6D(const Transform6D &other) = default;
    Transform6D(Transform6D &&other) = default;
    Transform6D &operator=(const Transform6D &other) = default;
    Transform6D &operator=(Transform6D &&other) = default;
    ~Transform6D() = default;

    /**
     * @brief 构造函数
     * @param rmat 旋转矩阵
     * @param tvec 平移向量
     */
    template <transform6D_concepts::rmat_type T, transform6D_concepts::tvec_type U>
    Transform6D(const T &rmat, const U &tvec);

    /**
     * @brief 构造函数
     * @param rvec 旋转向量
     * @param tvec 平移向量
     */
    template <transform6D_concepts::rvec_type T, transform6D_concepts::tvec_type U>
    Transform6D(const T &rvec, const U &tvec);

    /**
     * @brief 获取旋转矩阵
     * @return 返回旋转矩阵
     */
    const RmatType &rmat() const noexcept;

    /**
     * @brief 获取平移向量
     * @return 返回平移向量
     */
    const TvecType &tvec() const noexcept;

    /**
     * @brief 获取旋转向量
     * @return 返回旋转向量
     */
    const RvecType &rvec() const noexcept;

    /**
     * @brief 设置 rmat
     *
     * @param rmat 旋转矩阵
     * @return Transform6D& 返回当前对象的引用
     */
    template <transform6D_concepts::rmat_type T>
    void rmat(const T &rmat) noexcept;

    /**
     * @brief 设置 rvec
     *
     * @param rvec 旋转向量
     * @return Transform6D& 返回当前对象的引用
     */
    template <transform6D_concepts::rvec_type T>
    void rvec(const T &rvec) noexcept;

    /**
     * @brief 设置 tvec
     *
     * @param tvec 平移向量
     * @return Transform6D& 返回当前对象的引用
     */
    template <transform6D_concepts::tvec_type T>
    void tvec(const T &tvec) noexcept;

    /**
     * @brief 求逆变换
     * @return Transform6D 返回逆变换
     */
    Transform6D inv() const noexcept;

    //----------------------绕自身坐标轴旋转--------------------------------
    /**
     * @brief 绕自身X轴旋转
     * @param angle 旋转角度
     * @param mode 角度模式，默认为 DEG（度）
     * @note 旋转方向满足右手定则
     */
    void rotate_x(double angle, AngleMode mode = DEG);

    /**
     * @brief 绕自身Y轴旋转
     * @param angle 旋转角度
     * @param mode 角度模式，默认为 DEG（度）
     * @note 旋转方向满足右手定则
     */
    void rotate_y(double angle, AngleMode mode = DEG);

    /**
     * @brief 绕自身Z轴旋转
     * @param angle 旋转角度
     * @param mode 角度模式，默认为 DEG（度）
     * @note 旋转方向满足右手定则
     */
    void rotate_z(double angle, AngleMode mode = DEG);

private:
    // 确保旋转矩阵有效（延迟初始化）
    void ensureRmatInitialized() const;

    // 确保旋转向量有效（延迟初始化）
    void ensureRvecInitialized() const;

    mutable RmatType __rmat; //!< 旋转矩阵
    mutable RvecType __rvec; //!< 旋转向量
    TvecType __tvec;         //!< 平移向量

    mutable bool __rmat_initialized = false; //!< 旋转矩阵初始化标记
    mutable bool __rvec_initialized = false; //!< 旋转向量初始化标记
};

//! Transform6D 类的辅助函数
namespace transform6D_utils
{
    /**
     * @brief 将旋转矩阵转换为 Transform6D 所需的旋转矩阵类型
     * @tparam T 旋转矩阵类型
     * @param rmat 输入的旋转矩阵
     * @return 返回转换后的旋转矩阵
     */
    template <transform6D_concepts::rmat_type T>
    Transform6D::RmatType convertRmat(const T &rmat);

    /**
     * @brief 将旋转向量转换为 Transform6D 所需的旋转矩阵类型
     * @tparam T 旋转向量类型
     * @param rvec 输入的旋转向量
     * @return 返回转换后的旋转矩阵
     */
    template <transform6D_concepts::rvec_type T>
    Transform6D::RvecType convertRvec(const T &rvec);

    /**
     * @brief 将平移向量转换为 Transform6D 所需的平移向量类型
     * @tparam T 平移向量类型
     * @param tvec 输入的平移向量
     * @return 返回转换后的平移向量
     */
    template <transform6D_concepts::tvec_type T>
    Transform6D::TvecType convertTvec(const T &tvec);

    /**
     * @brief 将旋转向量转换为 Transform6D 所需的旋转矩阵类型
     * @param rvec 输入的旋转向量
     * @return 返回转换后的旋转矩阵
     */
    Transform6D::RmatType convertRmat(const cv::Matx<Transform6D::RmatType::value_type, 3, 1> &rvec);
}

// 默认构造函数
inline Transform6D::Transform6D()
    : __tvec(TvecType(0, 0, 0)),
      __rmat(RmatType::eye()),
      __rvec(RvecType(0, 0, 0))
{
}

// 使用旋转矩阵和平移向量构造
template <transform6D_concepts::rmat_type T, transform6D_concepts::tvec_type U>
inline Transform6D::Transform6D(const T &rmat, const U &tvec)
    : __rmat(transform6D_utils::convertRmat(rmat)),
      __tvec(transform6D_utils::convertTvec(tvec)),
      __rmat_initialized(true),
      __rvec_initialized(false)
{
    // 旋转矩阵已初始化，旋转向量未初始化
}

// 使用旋转向量和平移向量构造
template <transform6D_concepts::rvec_type T, transform6D_concepts::tvec_type U>
inline Transform6D::Transform6D(const T &rvec, const U &tvec)
    : __rvec(transform6D_utils::convertRvec(rvec)),
      __tvec(transform6D_utils::convertTvec(tvec)),
      __rvec_initialized(true),
      __rmat_initialized(false)
{
    // 旋转向量已初始化，旋转矩阵未初始化
}

// 确保旋转矩阵有效
inline void Transform6D::ensureRmatInitialized() const
{
    if (!__rmat_initialized)
    {
        if (__rvec_initialized)
        {
            // 从旋转向量计算旋转矩阵
            cv::Rodrigues(__rvec, __rmat);
        }
        else
        {
            // 默认单位矩阵
            __rmat = RmatType::eye();
        }
        __rmat_initialized = true;
    }
}

// 确保旋转向量有效
inline void Transform6D::ensureRvecInitialized() const
{
    if (!__rvec_initialized)
    {
        if (__rmat_initialized)
        {
            // 从旋转矩阵计算旋转向量
            cv::Rodrigues(__rmat, __rvec);
        }
        else
        {
            // 默认零向量
            __rvec = RvecType(0, 0, 0);
        }
        __rvec_initialized = true;
    }
}

inline const Transform6D::RmatType &Transform6D::rmat() const noexcept
{
    ensureRmatInitialized();
    return __rmat;
}

inline const Transform6D::TvecType &Transform6D::tvec() const noexcept
{
    return __tvec;
}

inline const Transform6D::RvecType &Transform6D::rvec() const noexcept
{
    ensureRvecInitialized();
    return __rvec;
}

template <transform6D_concepts::rmat_type T>
inline void Transform6D::rmat(const T &rmat) noexcept
{
    __rmat = transform6D_utils::convertRmat(rmat);
    __rmat_initialized = true;
    __rvec_initialized = false; // 旋转向量未初始化
}

template <transform6D_concepts::rvec_type T>
inline void Transform6D::rvec(const T &rvec) noexcept
{
    __rvec = transform6D_utils::convertRvec(rvec);
    __rvec_initialized = true;
    __rmat_initialized = false; // 旋转矩阵未初始化
}

template <transform6D_concepts::tvec_type T>
inline void Transform6D::tvec(const T &tvec) noexcept
{
    __tvec = transform6D_utils::convertTvec(tvec);
}

inline Transform6D Transform6D::inv() const noexcept
{
    ensureRmatInitialized();
    ensureRvecInitialized();
    // 计算逆变换
    return Transform6D(__rmat.t(), -__rmat.t() * __tvec);
}

/**
 * @brief Transform6D 加法操作
 * @param A_to_B A到B的变换
 * @param B_to_C B到C的变换
 * @return A_to_C A到C的变换
 */
inline Transform6D operator+(const Transform6D &A_to_B, const Transform6D &B_to_C)
{
    // 计算新的旋转矩阵和平移向量
    const Transform6D::RmatType new_rmat = A_to_B.rmat() * B_to_C.rmat();
    const Transform6D::TvecType new_tvec = A_to_B.rmat() * B_to_C.tvec() + A_to_B.tvec();

    return Transform6D(new_rmat, new_tvec);
}

/**
 * @brief Transform6D 减法操作
 * @param A_to_C A到C的变换
 * @param B_to_C B到C的变换
 * @return A_to_B A到B的变换
 */
inline Transform6D operator-(const Transform6D &A_to_C, const Transform6D &B_to_C)
{
    const auto C_to_B_rmat = B_to_C.rmat().t();            // B到C的逆变换
    const auto C_to_B_tvec = -C_to_B_rmat * B_to_C.tvec(); // B到C的逆平移向量
    // 计算新的旋转矩阵和平移向量
    const Transform6D::RmatType new_rmat = A_to_C.rmat() * C_to_B_rmat;
    const Transform6D::TvecType new_tvec = A_to_C.rmat() * C_to_B_tvec + A_to_C.tvec();

    return Transform6D(new_rmat, new_tvec);
}

/**
 * @brief Transform6D 叠加操作
 * @param A_to_B A到B的变换
 * @param B_to_C B到C的变换
 * @return A_to_C A到C的变换
 */
inline Transform6D &operator+=(Transform6D &A_to_B, const Transform6D &B_to_C)
{
    // 计算新的旋转矩阵和平移向量
    const Transform6D::RmatType new_rmat = A_to_B.rmat() * B_to_C.rmat();
    const Transform6D::TvecType new_tvec = A_to_B.rmat() * B_to_C.tvec() + A_to_B.tvec();

    A_to_B.rmat(new_rmat);
    A_to_B.tvec(new_tvec);
    return A_to_B;
}

/**
 * @brief Transform6D 减法赋值操作
 * @param A_to_C A到C的变换
 * @param B_to_C B到C的变换
 * @return A_to_B A到B的变换
 */
inline Transform6D &operator-=(Transform6D &A_to_C, const Transform6D &B_to_C)
{
    // 计算新的旋转矩阵和平移向量
    const auto C_to_B_rmat = B_to_C.rmat().t();            // B到C的逆变换
    const auto C_to_B_tvec = -C_to_B_rmat * B_to_C.tvec(); // B到C的逆平移向量
    const Transform6D::RmatType new_rmat = A_to_C.rmat() * C_to_B_rmat;
    const Transform6D::TvecType new_tvec = A_to_C.rmat() * C_to_B_tvec + A_to_C.tvec();
    A_to_C.rmat(new_rmat);
    A_to_C.tvec(new_tvec);

    return A_to_C;
}

inline void Transform6D::rotate_x(double angle, AngleMode mode)
{
    rmat(__rmat * euler2Mat(mode == DEG ? deg2rad(angle) : angle, EulerAxis::X));
}

inline void Transform6D::rotate_y(double angle, AngleMode mode)
{
    rmat(__rmat * euler2Mat(mode == DEG ? deg2rad(angle) : angle, EulerAxis::Y));
}

inline void Transform6D::rotate_z(double angle, AngleMode mode)
{
    rmat(__rmat * euler2Mat(mode == DEG ? deg2rad(angle) : angle, EulerAxis::Z));
}

namespace transform6D_utils
{
    /**
     * @brief 将任意符合 rmat_type 概念的旋转矩阵转换为 Transform6D::RmatType
     *
     * @tparam T 输入旋转矩阵类型
     * @param rmat 输入旋转矩阵
     * @return Transform6D::RmatType 转换后的旋转矩阵
     */
    template <transform6D_concepts::rmat_type T>
    inline Transform6D::RmatType convertRmat(const T &rmat)
    {
        return static_cast<Transform6D::RmatType>(rmat);
    }

    /**
     * @brief 将任意符合 rvec_type 概念的旋转向量转换为 Transform6D::RvecType
     *
     * @tparam T 输入旋转向量类型
     * @param rvec 输入旋转向量
     * @return Transform6D::RvecType 转换后的旋转向量
     */
    template <transform6D_concepts::rvec_type T>
    inline Transform6D::RvecType convertRvec(const T &rvec)
    {
        return geom_utils_concepts::convert3d<Transform6D::RvecType>(rvec);
    }

    /**
     * @brief 将任意符合 tvec_type 概念的平移向量转换为 Transform6D::TvecType
     *
     * @tparam T 输入平移向量类型
     * @param tvec 输入平移向量
     * @return Transform6D::TvecType 转换后的平移向量
     */
    template <transform6D_concepts::tvec_type T>
    inline Transform6D::TvecType convertTvec(const T &tvec)
    {
        return geom_utils_concepts::convert3d<Transform6D::TvecType>(tvec);
    }

    /**
     * @brief 将 3x1 旋转向量矩阵转换为 Transform6D::RmatType 旋转矩阵
     *
     * @param rvec 输入旋转向量（cv::Matx<value_type, 3, 1>）
     * @return Transform6D::RmatType 转换后的旋转矩阵
     */
    inline Transform6D::RmatType convertRmat(const cv::Matx<Transform6D::RvecType::value_type, 3, 1> &rvec)
    {
        auto rmat = Transform6D::RmatType::eye();
        cv::Rodrigues(rvec, rmat);
        return rmat;
    }
}
