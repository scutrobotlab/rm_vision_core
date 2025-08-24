/**
 * @file type_expansion.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 算术区间模板类及像素通道定义
 * @date 2025-7-15
 */

#pragma once
#include <iostream>
#include <type_traits>

/**
 * @class ArithmeticRange
 * @brief 算术区间模板类
 * @tparam T 算术类型，如 int、float、double
 *
 * @note 提供区间的长度计算、包含判断、交集判断、打印和运算符重载功能
 */
template <typename T>
    requires std::is_arithmetic_v<T> // 确保 T 是算术类型
class ArithmeticRange
{
public:
    T low;  //!< 区间下界
    T high; //!< 区间上界

    /**
     * @brief 默认构造函数
     *
     * @note 区间初始化为 T 类型的默认值
     */
    ArithmeticRange() : low(), high() {}

    /**
     * @brief 构造函数
     * @param low 区间下界
     * @param high 区间上界
     *
     * @note 如果 low > high，会自动交换保证 low <= high
     */
    ArithmeticRange(T low, T high) : low(low), high(high)
    {
        if (low > high)
        {
            std::swap(this->low, this->high); // 保证 low <= high
        }
    }

    /**
     * @brief 获取区间长度
     * @return 区间长度（high - low）
     */
    T length() const
    {
        return high - low;
    }

    /**
     * @brief 判断值是否在区间内
     * @param value 待判断值
     * @return true 如果 value 在区间内，否则 false
     */
    bool contains(T value) const
    {
        return value >= low && value <= high;
    }

    /**
     * @brief 判断区间是否与另一个区间有交集
     * @param other 另一个区间
     * @return true 如果两个区间有交集，否则 false
     */
    bool intersects(const ArithmeticRange<T> &other) const
    {
        return !(other.high < low || other.low > high);
    }

    /**
     * @brief 打印区间信息
     */
    void print() const
    {
        std::cout << "[" << low << ", " << high << "]" << std::endl;
    }

    /**
     * @brief 判断两个区间是否相等
     * @param other 另一个区间
     * @return true 如果区间上下界相等，否则 false
     */
    bool operator==(const ArithmeticRange<T> &other) const
    {
        return low == other.low && high == other.high;
    }

    /**
     * @brief 运算符重载：获取区间的宽度差
     * @param other 另一个区间
     * @return 当前区间下界减去另一区间上界
     */
    T operator-(const ArithmeticRange<T> &other) const
    {
        return this->low - other.high;
    }
};

// 区间类型别名
using RangeI = ArithmeticRange<int>;            //!< 整数区间
using RangeF = ArithmeticRange<float>;          //!< 浮点区间
using RangeD = ArithmeticRange<double>;         //!< 双精度浮点区间
using RangeSize = ArithmeticRange<std::size_t>; //!< 尺寸区间

/**
 * @enum PixChannel
 * @brief 像素通道枚举
 */
enum PixChannel : uint8_t
{
    BLUE = 0U,  //!< 蓝色通道
    GREEN = 1U, //!< 绿色通道
    RED = 2U    //!< 红色通道
};
