/**
 * @file rune_param.h
 * @brief 神符组合体参数模块头文件
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include "vc/core/yml_manager.hpp"

/**
 * @brief RuneParam 参数模块
 *
 * 用于管理神符整体相关的参数。
 */
struct RuneParam
{
private:
    YML_INIT(RuneParam, );
};

//! RuneParam 参数实例
inline RuneParam rune_param;
