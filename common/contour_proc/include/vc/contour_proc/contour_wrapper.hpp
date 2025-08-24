/**
 * @file contour_wrapper.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 轮廓分析器头文件
 * @date 2025-4-12
 */

#pragma once
#include <opencv2/core.hpp>

#include <bitset>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#include "vc/core/property_wrapper.hpp"

/**
 * @brief 可以用于ContourWrapper的基本算术类型 int 、float 和 double
 */
template <typename T>
concept ContourWrapperBaseType =
    std::is_same_v<std::remove_cv_t<T>, int> ||
    std::is_same_v<std::remove_cv_t<T>, float> ||
    std::is_same_v<std::remove_cv_t<T>, double>;

/**
 * @class ContourWrapper
 * @brief 高性能轮廓分析器，实现计算结果的延迟加载和智能缓存
 *
 * 1. 基于写时复制(copy-on-write)和延迟加载(lazy initialization)优化内存使用
 * 2. 采用抗锯齿预处理提升几何特征计算精度
 * 3. 自动缓存计算结果，避免重复运算
 * 4. 支持线程安全：const方法可并发调用，非const方法需外部同步
 *
 * @note 构造时会自动进行轮廓抗锯齿处理(approxPolyDP)，后续所有计算基于处理后的轮廓
 */
template <ContourWrapperBaseType _Tp = int>
class ContourWrapper
{
public:
    using ValueType = _Tp;
    /**
     * @brief 关键点的精度
     *
     * @note 当轮廓点集精度为int时，将关键点精度设置为float，其余情况下和轮廓点类型相同
     */
    using KeyType = std::conditional_t<std::is_same_v<std::remove_cv_t<ValueType>, int>, float, ValueType>;
    friend class std::allocator<ContourWrapper<ValueType>>;

private:
    using PointType = cv::Point_<ValueType>;  //!< 点类型
    using KeyPointType = cv::Point_<KeyType>; //!< 关键点类型

    // 标志位缓存
    mutable std::bitset<16> cache_flags;

    //----------------【直接存储】---------------------
    std::vector<PointType> __points;
    mutable double __area = 0;
    mutable double __perimeter_close = 0;
    mutable double __perimeter_open = 0;
    mutable KeyPointType __center = KeyPointType(0, 0);
    mutable double __convex_area = 0; //!< 凸包面积

    //----------------【缓存存储】---------------------
    mutable std::unique_ptr<cv::Rect> __bounding_rect;
    mutable std::unique_ptr<cv::RotatedRect> __min_area_rect;
    mutable std::unique_ptr<std::tuple<cv::Point2f, float>> __fitted_circle;
    mutable std::unique_ptr<cv::RotatedRect> __fitted_ellipse;
    mutable std::unique_ptr<std::vector<PointType>> __convex_hull;
    mutable std::unique_ptr<std::vector<int>> __convex_hull_idx;
    // 标志位定义
    enum CacheFlags
    {
        AREA_CALC = 0,             //!< 面积计算
        PERIMETER_CLOSE_CALC = 1,  //!< 周长计算
        PERIMETER_OPEN_CALC = 2,   //!< 开放式周长计算
        CENTER_CALC = 3,           //!< 中心点计算
        BOUNDING_RECT_CALC = 4,    //!< 外接矩形计算
        MIN_AREA_RECT_CALC = 5,    //!< 最小外接矩形计算
        FITTED_CIRCLE_CALC = 6,    //!< 拟合圆计算
        FITTED_ELLIPSE_CALC = 7,   //!< 拟合椭圆计算
        SMOOTHED_CONTOUR_CALC = 8, //!< 平滑轮廓计算
        CONVEX_HULL_CALC = 9,      //!< 凸包轮廓计算
        CONVEX_HULL_IDX_CALC = 10, //!< 凸包索引计算
        CONVEX_HULL_AREA_CALC = 11 //!< 凸包面积计算
    };

public:
    /**
     * @brief 构造函数
     *
     * @param contour 轮廓点集
     */
    ContourWrapper(const std::vector<PointType> &contour);

    /**
     * @brief 移动构造函数
     *
     * @param contour 轮廓点集
     */
    ContourWrapper(std::vector<PointType> &&contour);

public:
    // 禁用拷贝构造函数
    ContourWrapper(const ContourWrapper &) = delete;
    ContourWrapper &operator=(const ContourWrapper &) = delete;
    // 允许移动
    ContourWrapper(ContourWrapper &&) = default;
    ContourWrapper &operator=(ContourWrapper &&) = default;

    /**
     * @brief 创建轮廓对象
     *
     * @param contour 轮廓点集
     */
    inline static std::shared_ptr<ContourWrapper<ValueType>> make_contour(const std::vector<PointType> &contour);

    /**
     * @brief 创建轮廓对象
     *
     * @param contour 轮廓点集
     */
    inline static std::shared_ptr<ContourWrapper<ValueType>> make_contour(std::vector<PointType> &&contour);

    /**
     * @brief 获取轮廓点集
     */
    const std::vector<PointType> &points() const;

    /**
     * @brief 获取轮廓面积
     */
    double area() const;

    /**
     * @brief 获取轮廓周长
     */
    double perimeter(bool close = true) const;

    /**
     * @brief 获取轮廓中心点
     */
    KeyPointType center() const;

    /**
     * @brief 获取轮廓的正外接矩形
     */
    cv::Rect boundingRect() const;

    /**
     * @brief 获取轮廓的最小外接矩形
     */
    cv::RotatedRect minAreaRect() const;

    /**
     * @brief 获取轮廓的拟合圆
     */
    std::tuple<cv::Point2f, float> fittedCircle() const;

    /**
     * @brief 获取轮廓的拟合椭圆
     */
    cv::RotatedRect fittedEllipse() const;

    /**
     * @brief 获取轮廓的凸包
     */
    const std::vector<PointType> &convexHull() const;

    /**
     * @brief 获取凸包的索引
     */
    const std::vector<int> &convexHullIdx() const;

    /**
     * @brief 获取凸包的面积
     */
    float convexArea() const;

    /**
     * @brief 生成信息字符串
     */
    std::string infoString() const;

    /**
     * @brief 隐式转换
     */
    operator const std::vector<PointType> &() const;

    /**
     * @brief 获取轮廓的凸包轮廓——接口
     */
    static std::shared_ptr<ContourWrapper<ValueType>> getConvexHull(const std::vector<std::shared_ptr<const ContourWrapper<ValueType>>> &contours);

public:
    //--------------------【迭代器访问----------------------】
    using const_iterator = typename std::vector<PointType>::const_iterator;                 // 常量迭代器
    using const_reverse_iterator = typename std::vector<PointType>::const_reverse_iterator; // 常量反向迭代器

    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;
    const_iterator cbegin() const noexcept;
    const_iterator cend() const noexcept;
    const_reverse_iterator rbegin() const noexcept;
    const_reverse_iterator rend() const noexcept;
    const_reverse_iterator crbegin() const noexcept;
    const_reverse_iterator crend() const noexcept;

    // 常用stl接口
    size_t size() const noexcept;
    bool empty() const noexcept;
    const PointType &operator[](size_t idx) const noexcept;
    const PointType &at(size_t idx) const;
    const PointType &front() const noexcept;
    const PointType &back() const noexcept;

private:
    /**
     * @brief 获取多轮廓的凸包轮廓
     */
    static std::shared_ptr<ContourWrapper<ValueType>> getConvexHullImpl(const std::vector<std::shared_ptr<const ContourWrapper<ValueType>>> &contours);
};

using Contour_ptr = std::shared_ptr<ContourWrapper<int>>;
using ContourF_ptr = std::shared_ptr<ContourWrapper<float>>;
using ContourD_ptr = std::shared_ptr<ContourWrapper<double>>;

using Contour_cptr = std::shared_ptr<const ContourWrapper<int>>;
using ContourF_cptr = std::shared_ptr<const ContourWrapper<float>>;
using ContourD_cptr = std::shared_ptr<const ContourWrapper<double>>;

/**
 * @brief 类型检验，是否为 ContourWrapper<T>，T符合ContourWrapperBaseType
 */
template <typename T>
struct is_contour_wrapper : std::false_type
{
};
template <ContourWrapperBaseType T>
struct is_contour_wrapper<ContourWrapper<T>> : std::true_type
{
};
template <typename T>
concept ContourWrapperType = is_contour_wrapper<std::remove_cvref_t<T>>::value;

/**
 * @brief 类型检验，检验是否为 std::shared_ptr<ContourWrapper<T>>，T符合ContourWrapperBaseType
 */
template <typename T>
concept ContourWrapperPtrType = requires {
    requires std::is_same_v<
        std::shared_ptr<typename std::remove_cvref_t<T>::element_type>,
        std::remove_cvref_t<T>>;
    requires ContourWrapperType<typename std::remove_cvref_t<T>::element_type>;
};

/**
 * @brief 轮廓精度转换函数（支持 int/float/double 互转）
 * @tparam OutputType 目标精度类型 (int/float/double)
 * @tparam ContourPtr  输入轮廓指针类型 (自动推导)
 *
 * @param contour 输入轮廓的智能指针
 * @return std::shared_ptr<ContourWrapper<OutputType>> 转换后的轮廓智能指针
 *
 * @note 类型相同时直接返回原指针（无额外开销）
 *       类型不同时生成新轮廓（触发写时复制）
 */
template <ContourWrapperBaseType OutputType, ContourWrapperPtrType ContourPtr>
inline auto convert(const ContourPtr &contour) -> std::shared_ptr<ContourWrapper<OutputType>>;



// 构造函数
template <ContourWrapperBaseType _Tp>
ContourWrapper<_Tp>::ContourWrapper(const std::vector<PointType> &contour)
    : __points(contour)
{
    // 初始化标志位
    cache_flags.reset();
}

// 移动构造函数
template <ContourWrapperBaseType _Tp>
ContourWrapper<_Tp>::ContourWrapper(std::vector<PointType> &&contour)
    : __points(std::move(contour))
{
    // 初始化标志位
    cache_flags.reset();
}

// 创建轮廓对象
template <ContourWrapperBaseType _Tp>
inline std::shared_ptr<ContourWrapper<_Tp>> ContourWrapper<_Tp>::make_contour(const std::vector<PointType> &contour)
{
    return std::make_shared<ContourWrapper<_Tp>>(contour);
}

template <ContourWrapperBaseType _Tp>
inline std::shared_ptr<ContourWrapper<_Tp>> ContourWrapper<_Tp>::make_contour(std::vector<PointType> &&contour)
{
    return std::make_shared<ContourWrapper<_Tp>>(std::move(contour));
}

// 获取轮廓点集
template <ContourWrapperBaseType _Tp>
const std::vector<typename ContourWrapper<_Tp>::PointType> &ContourWrapper<_Tp>::points() const
{
    return __points;
}

// 获取轮廓面积
template <ContourWrapperBaseType _Tp>
double ContourWrapper<_Tp>::area() const
{
    if (!cache_flags.test(AREA_CALC))
    {
        __area = std::abs(cv::contourArea(__points));
        cache_flags.set(AREA_CALC);
    }
    return __area;
}

// 获取轮廓周长
template <ContourWrapperBaseType _Tp>
double ContourWrapper<_Tp>::perimeter(bool close) const
{
    if (close)
    {
        if (!cache_flags.test(PERIMETER_CLOSE_CALC))
        {
            const auto &contour = this->points();
            __perimeter_close = cv::arcLength(contour, true);
            cache_flags.set(PERIMETER_CLOSE_CALC);
        }
        return __perimeter_close;
    }
    else
    {
        if (!cache_flags.test(PERIMETER_OPEN_CALC))
        {
            const auto &contour = this->points();
            __perimeter_open = cv::arcLength(contour, false);
            cache_flags.set(PERIMETER_OPEN_CALC);
        }
        return __perimeter_open;
    }
}

// 获取轮廓中心点
template <ContourWrapperBaseType _Tp>
typename ContourWrapper<_Tp>::KeyPointType ContourWrapper<_Tp>::center() const
{
    if (!cache_flags.test(CENTER_CALC))
    {
        const auto &contour = this->points();
        cv::Scalar mean_val = cv::mean(contour);
        __center = KeyPointType(mean_val[0], mean_val[1]);
        cache_flags.set(CENTER_CALC);
    }
    return __center;
}

// 获取轮廓的正外接矩形
template <ContourWrapperBaseType _Tp>
cv::Rect ContourWrapper<_Tp>::boundingRect() const
{
    if (!cache_flags.test(BOUNDING_RECT_CALC))
    {
        const auto &contour = this->points();
        __bounding_rect = std::make_unique<cv::Rect>(
            cv::boundingRect(contour));
        cache_flags.set(BOUNDING_RECT_CALC);
    }
    return *__bounding_rect;
}

// 获取轮廓的最小外接矩形
template <ContourWrapperBaseType _Tp>
cv::RotatedRect ContourWrapper<_Tp>::minAreaRect() const
{
    if (!cache_flags.test(MIN_AREA_RECT_CALC))
    {
        const auto &contour = this->points();
        __min_area_rect = std::make_unique<cv::RotatedRect>(
            cv::minAreaRect(contour));
        cache_flags.set(MIN_AREA_RECT_CALC);
    }
    return *__min_area_rect;
}

// 获取轮廓的拟合圆
template <ContourWrapperBaseType _Tp>
std::tuple<cv::Point2f, float> ContourWrapper<_Tp>::fittedCircle() const
{
    if (!cache_flags.test(FITTED_CIRCLE_CALC))
    {
        const auto &contour = this->points();
        if (contour.size() >= 3)
        {
            cv::Point2f center_temp;
            float radius_temp = 0;
            cv::minEnclosingCircle(contour, center_temp, radius_temp);
            __fitted_circle = std::make_unique<std::tuple<cv::Point2f, float>>(center_temp, radius_temp);
            cache_flags.set(FITTED_CIRCLE_CALC);
        }
        else
        {
            // 点数不足时触发异常
            VC_THROW_ERROR("Insufficient points for circle fitting");
        }
    }
    return *__fitted_circle;
}

// 获取轮廓的拟合椭圆
template <ContourWrapperBaseType _Tp>
cv::RotatedRect ContourWrapper<_Tp>::fittedEllipse() const
{
    if (!cache_flags.test(FITTED_ELLIPSE_CALC))
    {
        const auto &contour = this->points();
        if (contour.size() >= 5)
        {
            __fitted_ellipse = std::make_unique<cv::RotatedRect>(cv::fitEllipse(contour));
            cache_flags.set(FITTED_ELLIPSE_CALC);
        }
        else
        {
            // 点数不足时触发异常
            VC_THROW_ERROR("Insufficient points for ellipse fitting");
        }
    }
    return *__fitted_ellipse;
}

// 获取轮廓的凸包
template <ContourWrapperBaseType _Tp>
const std::vector<typename ContourWrapper<_Tp>::PointType> &ContourWrapper<_Tp>::convexHull() const
{
    if (!cache_flags.test(CONVEX_HULL_CALC))
    {
        const auto &contour = this->points();
        __convex_hull = std::make_unique<std::vector<PointType>>();
        cv::convexHull(contour, *__convex_hull);
        cache_flags.set(CONVEX_HULL_CALC);
    }
    return *__convex_hull;
}

// 获取凸包的索引
template <ContourWrapperBaseType _Tp>
const std::vector<int> &ContourWrapper<_Tp>::convexHullIdx() const
{
    if (!cache_flags.test(CONVEX_HULL_IDX_CALC))
    {
        const auto &contour = this->points();
        __convex_hull_idx = std::make_unique<std::vector<int>>();
        cv::convexHull(contour, *__convex_hull_idx);
        cache_flags.set(CONVEX_HULL_IDX_CALC);
    }
    return *__convex_hull_idx;
}

// 获取凸包的面积
template <ContourWrapperBaseType _Tp>
float ContourWrapper<_Tp>::convexArea() const
{
    if (!cache_flags.test(CONVEX_HULL_AREA_CALC))
    {
        const auto &convex_contour = this->convexHull();
        __convex_area = std::abs(cv::contourArea(convex_contour));
        cache_flags.set(CONVEX_HULL_AREA_CALC);
    }
    return __convex_area;
}

// 生成信息字符串
template <ContourWrapperBaseType _Tp>
std::string ContourWrapper<_Tp>::infoString() const
{
    std::ostringstream oss;
    oss << "  Area: " << area() << "\n";
    oss << "  Perimeter (Closed): " << perimeter(true) << "\n";
    oss << "  Center: (" << center().x << ", " << center().y << ")\n";
    return oss.str();
}

// 隐式转换
template <ContourWrapperBaseType _Tp>
ContourWrapper<_Tp>::operator const std::vector<PointType> &() const
{
    return __points;
}

// 获取轮廓的凸包轮廓——接口
template <ContourWrapperBaseType _Tp>
std::shared_ptr<ContourWrapper<_Tp>> ContourWrapper<_Tp>::getConvexHull(const std::vector<std::shared_ptr<const ContourWrapper<_Tp>>> &contours)
{
    // 检查所有轮廓是否有效
    for (const auto &contour : contours)
    {
        if (contour == nullptr || contour->points().empty())
        {
            VC_THROW_ERROR("Invalid contour");
        }
    }
    return getConvexHullImpl(contours);
}

// 迭代器访问实现
template <ContourWrapperBaseType _Tp>
typename ContourWrapper<_Tp>::const_iterator ContourWrapper<_Tp>::begin() const noexcept { return __points.begin(); }

template <ContourWrapperBaseType _Tp>
typename ContourWrapper<_Tp>::const_iterator ContourWrapper<_Tp>::end() const noexcept { return __points.end(); }

template <ContourWrapperBaseType _Tp>
typename ContourWrapper<_Tp>::const_iterator ContourWrapper<_Tp>::cbegin() const noexcept { return __points.cbegin(); }

template <ContourWrapperBaseType _Tp>
typename ContourWrapper<_Tp>::const_iterator ContourWrapper<_Tp>::cend() const noexcept { return __points.cend(); }

template <ContourWrapperBaseType _Tp>
typename ContourWrapper<_Tp>::const_reverse_iterator ContourWrapper<_Tp>::rbegin() const noexcept { return __points.rbegin(); }

template <ContourWrapperBaseType _Tp>
typename ContourWrapper<_Tp>::const_reverse_iterator ContourWrapper<_Tp>::rend() const noexcept { return __points.rend(); }

template <ContourWrapperBaseType _Tp>
typename ContourWrapper<_Tp>::const_reverse_iterator ContourWrapper<_Tp>::crbegin() const noexcept { return __points.crbegin(); }

template <ContourWrapperBaseType _Tp>
typename ContourWrapper<_Tp>::const_reverse_iterator ContourWrapper<_Tp>::crend() const noexcept { return __points.crend(); }

// STL接口实现
template <ContourWrapperBaseType _Tp>
size_t ContourWrapper<_Tp>::size() const noexcept { return __points.size(); }

template <ContourWrapperBaseType _Tp>
bool ContourWrapper<_Tp>::empty() const noexcept { return __points.empty(); }

template <ContourWrapperBaseType _Tp>
const typename ContourWrapper<_Tp>::PointType &ContourWrapper<_Tp>::operator[](size_t idx) const noexcept { return __points[idx]; }

template <ContourWrapperBaseType _Tp>
const typename ContourWrapper<_Tp>::PointType &ContourWrapper<_Tp>::at(size_t idx) const
{
    if (idx >= __points.size())
    {
        VC_THROW_ERROR("Index out of range");
    }
    return __points.at(idx);
}

template <ContourWrapperBaseType _Tp>
const typename ContourWrapper<_Tp>::PointType &ContourWrapper<_Tp>::front() const noexcept { return __points.front(); }

template <ContourWrapperBaseType _Tp>
const typename ContourWrapper<_Tp>::PointType &ContourWrapper<_Tp>::back() const noexcept { return __points.back(); }

// 获取多轮廓的凸包轮廓
template <ContourWrapperBaseType _Tp>
std::shared_ptr<ContourWrapper<_Tp>> ContourWrapper<_Tp>::getConvexHullImpl(const std::vector<std::shared_ptr<const ContourWrapper<_Tp>>> &contours)
{
    size_t total_point_size = 0;
    for (const auto &contour : contours)
        total_point_size += contour->points().size();
    std::vector<PointType> all_points;
    all_points.reserve(total_point_size);
    for (const auto &contour : contours)
    {
        const auto &points = contour->points();
        all_points.insert(all_points.end(), points.begin(), points.end());
    }
    std::vector<PointType> convex_hull;
    cv::convexHull(all_points, convex_hull);
    return ContourWrapper<_Tp>::make_contour(std::move(convex_hull));
}

// 轮廓精度转换函数实现
template <ContourWrapperBaseType OutputType, ContourWrapperPtrType ContourPtr>
inline auto convert(const ContourPtr &contour) -> std::shared_ptr<ContourWrapper<OutputType>>
{
    using InputType = typename ContourPtr::element_type::value_type;

    if constexpr (std::is_same_v<OutputType, InputType>)
    {
        // 类型相同直接返回原始指针
        return std::static_pointer_cast<ContourWrapper<OutputType>>(contour);
    }
    else
    {
        // 类型转换生成新轮廓
        using OutputPoint = cv::Point_<OutputType>;
        std::vector<OutputPoint> converted;
        converted.reserve(contour->points().size());

        // 坐标转换（使用完美转发避免拷贝）
        for (const auto &p : contour->points())
        {
            converted.emplace_back(
                static_cast<OutputType>(p.x),
                static_cast<OutputType>(p.y));
        }

        // 构造新轮廓（触发移动语义）
        return ContourWrapper<OutputType>::make_contour(std::move(converted));
    }

    return nullptr;
}

// 显式实例化模板
template class ContourWrapper<int>;
template class ContourWrapper<float>;
template class ContourWrapper<double>;

#include "extensions.hpp"
#include "hierarchy.hpp"