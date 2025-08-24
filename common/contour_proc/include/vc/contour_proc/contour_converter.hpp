/**
 * @file contour_converter.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 轮廓转化模块
 * @date 2025-4-12
 */

#pragma once

#include "vc/contour_proc/contour_wrapper.hpp"

// 类型检验 concept

/**
 * @brief 能用于 ContourWrapper 转化的类型
 */
template <typename T>
concept ContourWrapperConvertable = requires(T t) {
    requires ContourWrapperPtrType<T>;
};
/**
 * @brief 轮廓转化模块
 *
 * @note 该模块用于将传入的各种格式的轮廓数据转换为 std::shared_ptr<ContourWrapper<int>> 对象
 */
class ContourConverter
{
private:
    /**
     * @brief 将传入的 ContourWrapperPtrType 类型转化为 std::shared_ptr<ContourWrapper<int>> 对象
     */
    template <ContourWrapperPtrType InputType>
    static Contour_ptr cvtContourWrapper(const InputType &contour)
    {
        auto contour_int = convert<int>(contour);
        return contour_int;
    }

public:
    /**
     * @brief 将传入类型转化为 std::shared_ptr<ContourWrapper<int>> 对象
     *
     * @param contours 输入的轮廓数据
     * @return std::shared_ptr<ContourWrapper<int>> 转换后的轮廓数据
     */
    template <ContourWrapperConvertable InputType>
    static Contour_ptr cvtContourWrapper(const InputType &contours)
    {
        // 如果是 ContourWrapperPtrType 类型，则直接转化
        if constexpr (ContourWrapperPtrType<InputType>)
        {
            return cvtContourWrapper(contours);
        }
        // 否则，抛出异常
        else
        {
            throw std::runtime_error("Unsupported contour type");
        }
    }
};