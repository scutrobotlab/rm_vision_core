/**
 * @file type_utils.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 类型转换工具
 * @date 2025-7-15
 */

#pragma once

#include <vector>
#include <list>
#include <deque>
#include <memory>
#include <tuple>

/**
 * @brief 常量智能指针类型别名
 * @tparam T 指针指向的类型
 */
template <typename T>
using shared_const_ptr = std::shared_ptr<const T>;

/**
 * @brief 将 vector<std::shared_ptr<T>> 转换为 vector<std::shared_ptr<const T>>
 * @tparam T 元素类型
 * @tparam Alloc 分配器类型，默认为 std::allocator<std::shared_ptr<T>>
 * @param c 输入 vector
 * @return 转换后的 vector，其中元素为 const 智能指针
 */
template <typename T, typename Alloc = std::allocator<std::shared_ptr<T>>>
std::vector<shared_const_ptr<T>> to_const(const std::vector<std::shared_ptr<T>, Alloc> &c)
{
    return std::vector<shared_const_ptr<T>>(c.begin(), c.end());
}

/**
 * @brief 将 list<std::shared_ptr<T>> 转换为 list<std::shared_ptr<const T>>
 * @tparam T 元素类型
 * @tparam Alloc 分配器类型，默认为 std::allocator<std::shared_ptr<T>>
 * @param c 输入 list
 * @return 转换后的 list，其中元素为 const 智能指针
 */
template <typename T, typename Alloc = std::allocator<std::shared_ptr<T>>>
std::list<shared_const_ptr<T>> to_const(const std::list<std::shared_ptr<T>, Alloc> &c)
{
    return std::list<shared_const_ptr<T>>(c.begin(), c.end());
}

/**
 * @brief 将 deque<std::shared_ptr<T>> 转换为 deque<std::shared_ptr<const T>>
 * @tparam T 元素类型
 * @tparam Alloc 分配器类型，默认为 std::allocator<std::shared_ptr<T>>
 * @param c 输入 deque
 * @return 转换后的 deque，其中元素为 const 智能指针
 */
template <typename T, typename Alloc = std::allocator<std::shared_ptr<T>>>
std::deque<shared_const_ptr<T>> to_const(const std::deque<std::shared_ptr<T>, Alloc> &c)
{
    return std::deque<shared_const_ptr<T>>(c.begin(), c.end());
}

/**
 * @brief 将 tuple<shared_ptr<T>...> 转换为 tuple<shared_ptr<const T>...>
 * @tparam Ts 元组中各元素类型
 * @param tpl 输入 tuple
 * @return 转换后的 tuple，其中每个元素为 const 智能指针
 */
template <typename... Ts>
auto to_const(const std::tuple<std::shared_ptr<Ts>...> &tpl)
{
    return std::apply([](auto const &...elems)
                      { return std::make_tuple(shared_const_ptr<Ts>(elems)...); }, tpl);
}

/**
 * @brief 将 vector<tuple<shared_ptr<Ts>...>> 转换为 vector<tuple<shared_ptr<const Ts>...>>
 * @tparam Ts 元组中各元素类型
 * @param vec 输入 vector
 * @return 转换后的 vector，其中每个 tuple 内元素均为 const 智能指针
 */
template <typename... Ts>
auto to_const(const std::vector<std::tuple<std::shared_ptr<Ts>...>> &vec)
{
    std::vector<std::tuple<shared_const_ptr<Ts>...>> result;
    result.reserve(vec.size());

    for (auto const &t : vec)
    {
        result.push_back(to_const(t));
    }
    return result;
}
