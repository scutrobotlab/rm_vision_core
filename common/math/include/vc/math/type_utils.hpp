/**
 * @file type_utils.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 几何向量类型概念及通用操作工具头文件
 * @date 2025-7-15
 */

#pragma once

#include <type_traits>
#include <opencv2/core.hpp>

namespace geom_utils_concepts
{

    /**
     * @brief 去除类型的 const 和引用限定
     */
    template <typename T>
    using base_type = std::remove_cvref_t<T>;

    /**
     * @brief 常用算术类型概念
     *
     * @note 支持 int, float, double 类型
     */
    template <typename T>
    concept vector_arithmetic = std::is_arithmetic_v<base_type<T>> &&
                                (std::is_same_v<base_type<T>, int> ||
                                 std::is_same_v<base_type<T>, float> ||
                                 std::is_same_v<base_type<T>, double>);

    // ---------------------------【二维向量】---------------------------

    /**
     * @brief 二维向量类型概念_1
     *
     * @note 拥有 x, y 成员，且为算术类型，不含 z 成员
     */
    template <typename T>
    concept vector_2_type_sub_1 = requires(T a) {
        a.x;
        a.y;
        requires vector_arithmetic<decltype(a.x)>;
        requires vector_arithmetic<decltype(a.y)>;
    } && !requires(T a) { a.z; };

    /**
     * @brief 二维向量类型概念_2
     *
     * @note 可通过 () 运算符访问元素，行为类似 1x2 矩阵
     */
    template <typename T>
    concept vector_2_type_sub_2 = requires(T a) {
        a(0, 0);
        a(0, 1);
        requires vector_arithmetic<decltype(a(0, 0))>;
        requires vector_arithmetic<decltype(a(0, 1))>;
        requires(T::rows == 1 && T::cols == 2);
    };

    /**
     * @brief 二维向量类型概念_3
     *
     * @note 可通过 () 运算符访问元素，行为类似 2x1 矩阵
     */
    template <typename T>
    concept vector_2_type_sub_3 = requires(T a) {
        a(0, 0);
        a(1, 0);
        requires vector_arithmetic<decltype(a(0, 0))>;
        requires vector_arithmetic<decltype(a(1, 0))>;
        requires(T::rows == 2 && T::cols == 1);
    };

    /**
     * @brief 二维向量类型概念_4
     *
     * @note cv::Vec<U,2> 类型，U 为基本算术类型
     */
    template <typename T>
    concept vector_2_type_sub_4 = requires(T a) {
        requires std::is_same_v<T, cv::Vec<typename T::value_type, 2>>;
        requires vector_arithmetic<typename T::value_type>;
    };

    /**
     * @brief 二维向量类型概念（综合）
     */
    template <typename T>
    concept vector_2_type = vector_2_type_sub_1<T> || vector_2_type_sub_2<T> ||
                            vector_2_type_sub_3<T> || vector_2_type_sub_4<T>;

    // ---------------------------【三维向量】---------------------------

    /**
     * @brief 三维向量类型概念_1
     *
     * @note 拥有 x, y, z 成员，且为算术类型
     */
    template <typename T>
    concept vector_3_type_sub_1 = requires(T a) {
        a.x;
        a.y;
        a.z;
        requires vector_arithmetic<decltype(a.x)>;
        requires vector_arithmetic<decltype(a.y)>;
        requires vector_arithmetic<decltype(a.z)>;
    };

    /**
     * @brief 三维向量类型概念_2
     *
     * @note 行向量 1x3，可通过 () 运算符访问元素
     */
    template <typename T>
    concept vector_3_type_sub_2 = requires(T a) {
        a(0, 0);
        a(0, 1);
        a(0, 2);
        requires vector_arithmetic<decltype(a(0, 0))>;
        requires vector_arithmetic<decltype(a(0, 1))>;
        requires vector_arithmetic<decltype(a(0, 2))>;
        requires(T::rows == 1 && T::cols == 3);
    };

    /**
     * @brief 三维向量类型概念_3
     *
     * @note 列向量 3x1，可通过 () 运算符访问元素
     */
    template <typename T>
    concept vector_3_type_sub_3 = requires(T a) {
        a(0, 0);
        a(1, 0);
        a(2, 0);
        requires vector_arithmetic<decltype(a(0, 0))>;
        requires vector_arithmetic<decltype(a(1, 0))>;
        requires vector_arithmetic<decltype(a(2, 0))>;
        requires(T::rows == 3 && T::cols == 1);
    };

    /**
     * @brief 三维向量类型概念_4
     *
     * @note cv::Vec<U,3> 类型，U 为基本算术类型
     */
    template <typename T>
    concept vector_3_type_sub_4 = requires {
        requires std::is_same_v<T, cv::Vec<typename T::value_type, 3>>;
        requires vector_arithmetic<typename T::value_type>;
    };

    /**
     * @brief 三维向量类型概念（综合）
     */
    template <typename T>
    concept vector_3_type = vector_3_type_sub_1<T> || vector_3_type_sub_2<T> ||
                            vector_3_type_sub_3<T> || vector_3_type_sub_4<T>;

    // ---------------------------【辅助模板】---------------------------
    namespace details
    {
        template <typename>
        constexpr bool always_false = false; //!< 用于静态断言
    }

    /**
     * @brief 提取向量 x 分量（支持 2D/3D 向量）
     * @tparam T 向量类型（需满足 vector_2_type 或 vector_3_type）
     * @param v 向量实例
     * @return x 分量值
     */
    template <typename T>
        requires(vector_2_type<T> || vector_3_type<T>)
    constexpr auto get_x(const T &v) noexcept
    {
        if constexpr (vector_2_type_sub_1<T> || vector_3_type_sub_1<T>)
            return v.x;
        else if constexpr (vector_2_type_sub_2<T>)
            return v(0, 0);
        else if constexpr (vector_2_type_sub_3<T> || vector_3_type_sub_3<T>)
            return v(0, 0);
        else if constexpr (vector_2_type_sub_4<T> || vector_3_type_sub_4<T>)
            return v[0];
        else if constexpr (vector_3_type_sub_2<T>)
            return v(0, 0);
        else
            static_assert(details::always_false<T>, "Unsupported type for get_x");
    }

    /**
     * @brief 提取向量 y 分量（支持 2D/3D 向量）
     * @tparam T 向量类型（需满足 vector_2_type 或 vector_3_type）
     * @param v 向量实例
     * @return y 分量值
     */
    template <typename T>
        requires(vector_2_type<T> || vector_3_type<T>)
    constexpr auto get_y(const T &v) noexcept
    {
        if constexpr (vector_2_type_sub_1<T> || vector_3_type_sub_1<T>)
            return v.y;
        else if constexpr (vector_2_type_sub_2<T>)
            return v(0, 1);
        else if constexpr (vector_2_type_sub_3<T>)
            return v(1, 0);
        else if constexpr (vector_3_type_sub_3<T>)
            return v(1, 0);
        else if constexpr (vector_2_type_sub_4<T> || vector_3_type_sub_4<T>)
            return v[1];
        else if constexpr (vector_3_type_sub_2<T>)
            return v(0, 1);
        else
            static_assert(details::always_false<T>, "Unsupported type for get_y");
    }

    /**
     * @brief 提取向量 z 分量（仅支持 3D 向量）
     * @tparam T 向量类型（需满足 vector_3_type）
     * @param v 3D 向量实例
     * @return z 分量值
     */
    template <typename T>
        requires vector_3_type<T>
    constexpr auto get_z(const T &v) noexcept
    {
        if constexpr (vector_3_type_sub_1<T>)
            return v.z;
        else if constexpr (vector_3_type_sub_2<T>)
            return v(0, 2);
        else if constexpr (vector_3_type_sub_3<T>)
            return v(2, 0);
        else if constexpr (vector_3_type_sub_4<T>)
            return v[2];
        else
            static_assert(details::always_false<T>,
                          "Unsupported type for get_z (must be 3D vector)");
    }

    /**
     * @brief 将任意 3D 向量转换为 cv::Matx<Tp,3,1> 类型
     * @tparam Tp 目标数据类型
     * @tparam T 输入向量类型（需满足 vector_3_type）
     * @param v 输入向量实例
     * @return 转换后的 cv::Matx<Tp,3,1>
     */
    template <typename Tp, typename T>
        requires vector_3_type<T>
    constexpr auto cvtMatx31(const T &v) noexcept
    {
        return cv::Matx<Tp, 3, 1>(get_x(v), get_y(v), get_z(v));
    }

    /**
     * @brief 将任意 3D 向量转换为 cv::Vec<Tp,3> 类型
     * @tparam Tp 目标数据类型
     * @tparam T 输入向量类型（需满足 vector_3_type）
     * @param v 输入向量实例
     * @return 转换后的 cv::Vec<Tp,3>
     */
    template <typename Tp, typename T>
        requires vector_3_type<T>
    constexpr auto cvtVec3(const T &v) noexcept
    {
        return cv::Vec<Tp, 3>(get_x(v), get_y(v), get_z(v));
    }

    /**
     * @brief 通用 3D 向量类型转换
     * @tparam Target 目标向量类型（cv::Vec<Tp,3> 或 cv::Matx<Tp,3,1>）
     * @tparam Source 源向量类型（需满足 vector_3_type）
     * @param v 待转换向量
     * @return 转换后的 Target 类型向量
     */
    template <typename Target, typename Source>
        requires vector_3_type<Source> &&
                 (std::same_as<std::remove_cvref_t<Target>, cv::Vec<typename Target::value_type, 3>> ||
                  std::same_as<std::remove_cvref_t<Target>, cv::Matx<typename Target::value_type, 3, 1>>)
    constexpr Target convert3d(const Source &v) noexcept
    {
        using Tp = typename Target::value_type;
        Tp x = get_x(v), y = get_y(v), z = get_z(v);
        if constexpr (std::same_as<std::remove_cvref_t<Target>, cv::Vec<Tp, 3>>)
            return Target{x, y, z};
        else
            return Target{x, y, z};
    }

} // namespace geom_utils_concepts
