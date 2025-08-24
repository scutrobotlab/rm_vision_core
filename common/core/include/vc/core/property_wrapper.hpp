/**
 * @file property_wrapper.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 属性包装类与相关宏定义
 * @date 2025-7-15
 */

#pragma once

#include <memory>
#include <string>
#include "vc/core/logging.h"

/**
 * @brief 参数展开辅助宏
 *
 * @note 用于展开被括号包裹的参数
 */
#define PROPERTY_WRAPPER_EXPAND_PARAMS(...) __VA_ARGS__

/**
 * @brief 核心属性包装类
 * @tparam WrapperType 被包装的属性类型
 *
 * @note 提供属性值的存在状态跟踪、安全访问、类型转发等功能
 */
template <typename WrapperType>
class PropertyWrapper
{
    WrapperType _value;      //!< 属性值
    bool _has_value = false; //!< 属性是否已设置

public:
    /**
     * @brief 默认构造函数
     */
    PropertyWrapper() = default;

    /**
     * @brief 移动构造函数
     * @param value 待移动的属性值
     */
    PropertyWrapper(WrapperType &&value) : _value(std::move(value)), _has_value(true) {}

    /**
     * @brief 拷贝构造函数
     * @param value 待拷贝的属性值
     */
    PropertyWrapper(const WrapperType &value) : _value(value), _has_value(true) {}

    /**
     * @brief 获取属性值（常量引用）
     * @return 属性值
     *
     * @note 如果属性未设置，将抛出异常
     */
    [[nodiscard]] const WrapperType &getValue() const
    {
        if (!_has_value)
            VC_THROW_ERROR("属性未设置");
        return _value;
    }

    /**
     * @brief 获取属性值（可修改引用）
     * @return 属性值
     *
     * @note 如果属性未设置，将抛出异常
     */
    WrapperType &getValue()
    {
        if (!_has_value)
            VC_THROW_ERROR("属性未设置");
        return _value;
    }

    /**
     * @brief 检查属性是否已设置
     * @return true 如果属性已设置，否则 false
     */
    [[nodiscard]] bool hasValue() const noexcept { return _has_value; }

    /**
     * @brief 设置属性值
     * @tparam U 可转为 WrapperType 的类型
     * @param value 待设置的值
     */
    template <typename U>
    void setValue(U &&value)
    {
        _value = std::forward<U>(value);
        _has_value = true;
    }

    /**
     * @brief 清除属性值
     */
    void clearValue() noexcept
    {
        _has_value = false;
    }
};

/**
 * @brief 定义属性宏
 *
 * @param PROPERTY_NAME 属性名
 * @param READ_SCOPE 读取访问权限
 * @param WRITE_SCOPE 写入访问权限
 * @param TYPE 属性类型
 *
 * @note 会生成 get/set/isSet/clear 方法
 */
#define DEFINE_PROPERTY(PROPERTY_NAME, READ_SCOPE, WRITE_SCOPE, TYPE)    \
private:                                                                 \
    using _##PROPERTY_NAME##_type = PROPERTY_WRAPPER_EXPAND_PARAMS TYPE; \
    PropertyWrapper<_##PROPERTY_NAME##_type>                             \
        _##PROPERTY_NAME##_prop;                                         \
    READ_SCOPE:                                                          \
    const _##PROPERTY_NAME##_type &get##PROPERTY_NAME() const            \
    {                                                                    \
        return _##PROPERTY_NAME##_prop.getValue();                       \
    }                                                                    \
    _##PROPERTY_NAME##_type &get##PROPERTY_NAME()                        \
    {                                                                    \
        return _##PROPERTY_NAME##_prop.getValue();                       \
    }                                                                    \
    bool isSet##PROPERTY_NAME() const noexcept                           \
    {                                                                    \
        return _##PROPERTY_NAME##_prop.hasValue();                       \
    }                                                                    \
                                                                         \
    WRITE_SCOPE:                                                         \
    template <typename U>                                                \
    void set##PROPERTY_NAME(U &&value)                                   \
    {                                                                    \
        _##PROPERTY_NAME##_prop.setValue(std::forward<U>(value));        \
    }                                                                    \
    void clear##PROPERTY_NAME() noexcept                                 \
    {                                                                    \
        _##PROPERTY_NAME##_prop.clearValue();                            \
    }

/**
 * @brief 定义带初始值的属性宏
 *
 * @param PROPERTY_NAME 属性名
 * @param READ_SCOPE 读取访问权限
 * @param WRITE_SCOPE 写入访问权限
 * @param TYPE 属性类型
 * @param VALUE 初始值
 *
 * @note 会生成 get/set/isSet/clear 方法，并在声明时初始化
 */
#define DEFINE_PROPERTY_WITH_INIT(PROPERTY_NAME, READ_SCOPE, WRITE_SCOPE, TYPE, VALUE...) \
private:                                                                                  \
    using _##PROPERTY_NAME##_type = PROPERTY_WRAPPER_EXPAND_PARAMS TYPE;                  \
    PropertyWrapper<_##PROPERTY_NAME##_type>                                              \
        _##PROPERTY_NAME##_prop{VALUE};                                                   \
    READ_SCOPE:                                                                           \
    const _##PROPERTY_NAME##_type &get##PROPERTY_NAME() const                             \
    {                                                                                     \
        return _##PROPERTY_NAME##_prop.getValue();                                        \
    }                                                                                     \
    _##PROPERTY_NAME##_type &get##PROPERTY_NAME()                                         \
    {                                                                                     \
        return _##PROPERTY_NAME##_prop.getValue();                                        \
    }                                                                                     \
    bool isSet##PROPERTY_NAME() const noexcept                                            \
    {                                                                                     \
        return _##PROPERTY_NAME##_prop.hasValue();                                        \
    }                                                                                     \
                                                                                          \
    WRITE_SCOPE:                                                                          \
    template <typename U>                                                                 \
    void set##PROPERTY_NAME(U &&value)                                                    \
    {                                                                                     \
        _##PROPERTY_NAME##_prop.setValue(std::forward<U>(value));                         \
    }                                                                                     \
    void clear##PROPERTY_NAME() noexcept                                                  \
    {                                                                                     \
        _##PROPERTY_NAME##_prop.clearValue();                                             \
    }
