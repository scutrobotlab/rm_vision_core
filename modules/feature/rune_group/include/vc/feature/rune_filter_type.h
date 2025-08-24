/**
 * @file rune_filter_type.h
 * @brief 神符滤波器目标数据类型枚举及运算符重载
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include <type_traits>

/**
 * @brief 神符滤波目标数据类型枚举
 *
 * 用于标识滤波器处理的数据类型。
 */
enum class RuneFilterDataType : unsigned int
{
    XYZ = 1 << 0,       //!< 仅滤波位移数据 x, y, z
    YAW_PITCH = 1 << 1, //!< 仅滤波 yaw 和 pitch 角
    ROLL = 1 << 2,      //!< 仅滤波 roll 角
};

/**
 * @brief 重载运算符 | (组合标志位)
 * @param[in] a 第一个数据类型标志
 * @param[in] b 第二个数据类型标志
 * @return RuneFilterDataType 组合后的标志
 */
inline RuneFilterDataType operator|(RuneFilterDataType a, RuneFilterDataType b)
{
    using underlying = std::underlying_type_t<RuneFilterDataType>;
    return static_cast<RuneFilterDataType>(static_cast<underlying>(a) | static_cast<underlying>(b));
}

/**
 * @brief 重载运算符 & (检查标志位)
 * @param[in] a 第一个数据类型标志
 * @param[in] b 第二个数据类型标志
 * @return RuneFilterDataType 位与结果
 */
inline RuneFilterDataType operator&(RuneFilterDataType a, RuneFilterDataType b)
{
    using underlying = std::underlying_type_t<RuneFilterDataType>;
    return static_cast<RuneFilterDataType>(static_cast<underlying>(a) & static_cast<underlying>(b));
}

/**
 * @brief 重载运算符 ~ (取反)
 * @param[in] a 数据类型标志
 * @return RuneFilterDataType 取反结果
 */
inline RuneFilterDataType operator~(RuneFilterDataType a)
{
    using underlying = std::underlying_type_t<RuneFilterDataType>;
    return static_cast<RuneFilterDataType>(~static_cast<underlying>(a));
}
