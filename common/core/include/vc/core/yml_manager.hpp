/**
 * @file yml_manager.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief YML 文件生成、加载和写入管理工具
 * @date 2025-7-15
 */

#pragma once

#include "vc/core/logging.h"
#include "vc/core/type_expansion.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>

// ---------------------------【调试宏】---------------------------
#define YML_DEBUG 1
#if YML_DEBUG
#define YML_DEBUG_PASS_(msg...) VC_PASS_INFO(msg)
#define YML_DEBUG_WARNING_(msg...) VC_WARNING_INFO(msg)
#define YML_DEBUG_INFO_(msg...) VC_NORMAL_INFO(msg)
#else
#define YML_DEBUG_PASS_(msg...)
#define YML_DEBUG_WARNING_(msg...)
#define YML_DEBUG_INFO_(msg...)
#endif

namespace yml_manager
{
    /**
     * @brief 根据源文件路径与类名生成 YML 文件路径
     *
     * @param source_file_path 头文件路径
     * @param class_name 类名
     * @return std::string 生成的 YML 文件路径
     */
    inline std::string generateYmlPath(const std::string &source_file_path, const std::string &class_name)
    {
        namespace fs = std::filesystem;

        fs::path header_dir = fs::path(source_file_path).parent_path();
        fs::path yml_path = header_dir / "yml" / (class_name + ".yml");

        if (!fs::exists(yml_path.parent_path()))
        {
            fs::create_directories(yml_path.parent_path());
            YML_DEBUG_INFO_("自动创建配置目录: %s", yml_path.parent_path().string().c_str());
        }

        return yml_path.string();
    }

} // namespace yml_manager

// ---------------------------【类名宏】---------------------------
#define YML_MANAGER_CURRENT_CLASS_NAME_ \
    ([]() -> const std::string & {                                       \
        static const std::string result = [] {                           \
            std::string_view prettyFunction = __PRETTY_FUNCTION__;       \
            size_t paramStart = prettyFunction.find('(');                \
            if (paramStart == std::string_view::npos)                    \
                return std::string("unknown");                           \
            size_t funcNameEnd = prettyFunction.rfind("::", paramStart); \
            if (funcNameEnd == std::string_view::npos)                   \
                return std::string("unknown");                           \
            size_t classNameStart = prettyFunction.rfind(' ', funcNameEnd) + 1; \
            return std::string(prettyFunction.substr(classNameStart, funcNameEnd - classNameStart)); \
        }();                                                              \
        return result; })()

// ---------------------------【YML 文件路径宏】---------------------------
#define YML_MANAGER_PARAM_FILE_PATH_ \
    yml_manager::generateYmlPath(__FILE__, YML_MANAGER_CURRENT_CLASS_NAME_)

// ---------------------------【构造函数宏】---------------------------
#define YML_MANAGER_CONSTRUCTOR_INIT(ParamStruct) \
    ParamStruct()                                 \
    {                                             \
        file_path = YML_MANAGER_PARAM_FILE_PATH_; \
        load(file_path);                          \
    }

// ---------------------------【YML 类型枚举】---------------------------
enum YmlType : unsigned int
{
    READ = 1 << 0,
    WRITE = 1 << 1
};

// ---------------------------【YML 文件检查】---------------------------
/**
 * @brief 检查 YML 文件格式是否正确（首行 %YAML:1.0）
 *
 * @param file_path 文件路径
 * @return true 格式正确
 * @return false 文件不存在或格式错误
 */
inline bool checkYmlFile(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        YML_DEBUG_WARNING_("yml文件打开失败: %s", file_path.c_str());
        return false;
    }
    std::string line;
    std::getline(file, line);
    if (line != "%YAML:1.0")
    {
        YML_DEBUG_WARNING_("yml文件格式错误: %s", file_path.c_str());
        return false;
    }
    return true;
}

// ---------------------------【参数加载初始化宏】---------------------------
#define YML_INIT(ParamStruct, add_param_function...)                                                            \
public:                                                                                                         \
    YML_MANAGER_CONSTRUCTOR_INIT(ParamStruct)                                                                   \
    std::string file_path;                                                                                      \
                                                                                                                \
public:                                                                                                         \
    void load(const cv::String &file_path, YmlType type = static_cast<YmlType>(YmlType::READ | YmlType::WRITE)) \
    {                                                                                                           \
        if (file_path.empty())                                                                                  \
        {                                                                                                       \
            YML_DEBUG_WARNING_("yml文件路径为空: %s", file_path.c_str());                                       \
            return;                                                                                             \
        }                                                                                                       \
        if (type & YmlType::READ)                                                                               \
        {                                                                                                       \
            cv::FileStorage fs(file_path, cv::FileStorage::READ);                                               \
            if (!fs.isOpened())                                                                                 \
            {                                                                                                   \
                YML_DEBUG_PASS_("yml 文件不存在,创建并初始化: %s", file_path.c_str());                          \
            }                                                                                                   \
            bool is_read = true;                                                                                \
            add_param_function                                                                                  \
        }                                                                                                       \
        if (type & YmlType::WRITE)                                                                              \
        {                                                                                                       \
            cv::FileStorage fs(file_path, cv::FileStorage::WRITE);                                              \
            if (!fs.isOpened())                                                                                 \
            {                                                                                                   \
                YML_DEBUG_WARNING_("yml文件打开失败: %s", file_path.c_str());                                   \
                return;                                                                                         \
            }                                                                                                   \
            bool is_read = false;                                                                               \
            add_param_function                                                                                  \
        }                                                                                                       \
    }

// ---------------------------【参数操作宏】---------------------------
#define YML_ADD_PARAM(param)               \
    do                                     \
    {                                      \
        if (is_read)                       \
        {                                  \
            readParam(fs, #param, param);  \
        }                                  \
        else                               \
        {                                  \
            writeParam(fs, #param, param); \
        }                                  \
    } while (false)

// ---------------------------【参数读写模板】---------------------------
template <typename _Tp>
inline void readParam(const cv::FileStorage &fs, const cv::String &name, _Tp &param)
{
    fs[name].isNone() ? void(0) : fs[name] >> param;
}

template <>
inline void readParam<size_t>(const cv::FileStorage &fs, const cv::String &name, size_t &param)
{
    int temp = 0;
    fs[name].isNone() ? void(0) : fs[name] >> temp;
    param = static_cast<size_t>(temp);
}

template <typename _Tp>
inline void writeParam(cv::FileStorage &fs, const cv::String &name, const _Tp &param)
{
    fs << name << param;
}

template <>
inline void writeParam<size_t>(cv::FileStorage &fs, const cv::String &name, const size_t &param)
{
    fs << name << static_cast<int>(param);
}

// ---------------------------【YmlManager 类】---------------------------
/**
 * @brief YML 文件管理器类
 */
class YmlManager
{
public:
    YmlManager() = default;
    ~YmlManager() = default;
};
