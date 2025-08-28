#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <sstream>
#include <stdexcept>

/**
 * @brief 命令行参数管理器
 *
 * 提供简洁的接口，用于：
 * - 添加短选项（如 `-i`）和长选项（如 `--input`）
 * - 解析命令行参数
 * - 查询参数是否被设置、获取参数值或默认值
 * - 自动打印帮助信息
 */
class CommandLine
{
public:
    /**
     * @brief 命令行选项结构
     */
    struct Option
    {
        std::string longName;     ///< 长选项名称，例如 `input`
        std::string description;  ///< 参数描述，用于帮助信息
        std::string value;        ///< 实际解析出的值
        std::string defaultValue; ///< 默认值（若用户未输入）
        bool requiresValue;       ///< 是否需要携带值
        bool isSet = false;       ///< 用户是否设置了该参数
    };

    /**
     * @brief 添加一个选项
     *
     * @param shortName 短选项名，如 "-i"
     * @param longName 长选项名，如 "input"
     * @param description 参数的说明信息
     * @param requiresValue 是否需要值，默认为 false
     * @param defaultValue 默认值，默认为空
     *
     * 示例：
     * @code
     * cli.addOption("-i", "input", "输入文件路径", true);
     * cli.addOption("-v", "verbose", "启用详细输出");
     * @endcode
     */
    void addOption(const std::string &shortName,
                   const std::string &longName,
                   const std::string &description,
                   bool requiresValue = false,
                   const std::string &defaultValue = "");

    /**
     * @brief 解析命令行参数
     *
     * @param argc main函数的参数数量
     * @param argv main函数的参数指针数组
     * @return true 如果解析成功
     *
     * @throws std::runtime_error 如果存在未知选项或缺少参数值
     *
     * 示例：
     * @code
     * cli.parse(argc, argv);
     * @endcode
     */
    bool parse(int argc, char *argv[]);

    /**
     * @brief 获取选项的值
     *
     * @param shortName 短选项名，如 "-i"
     * @return 对应的值。如果用户没有输入，则返回默认值；如果没有默认值，则返回空字符串
     *
     * 示例：
     * @code
     * std::string input = cli.get("-i");
     * @endcode
     */
    std::string get(const std::string &shortName) const;

    /**
     * @brief 判断选项是否被设置
     *
     * @param shortName 短选项名，如 "-v"
     * @return true 表示该选项被用户设置过
     *
     * 示例：
     * @code
     * if (cli.isSet("-v")) { std::cout << "Verbose mode on\n"; }
     * @endcode
     */
    bool isSet(const std::string &shortName) const;

    /**
     * @brief 打印帮助信息
     *
     * @param progName 程序名称（通常为 argv[0]）
     *
     * 示例：
     * @code
     * cli.printHelp(argv[0]);
     * @endcode
     *
     * 输出示例：
     * @verbatim
     * Usage: ./my_program [options]
     *   -i, --input <value>
     *       输入文件路径
     *   -v, --verbose
     *       启用详细输出
     * @endverbatim
     */
    void printHelp(const std::string &progName) const;

    /**
     * @brief 获取未被识别为选项的“位置参数”
     *
     * @return 所有位置参数的字符串数组
     *
     * 示例：
     * ```bash
     * ./my_program input1.txt input2.txt
     * ```
     * ```cpp
     * auto files = cli.getPositionalArgs();
     * ```
     */
    const std::vector<std::string> &getPositionalArgs() const;

private:
    std::unordered_map<std::string, Option> shortOptions_;     ///< 短选项映射
    std::unordered_map<std::string, std::string> longOptions_; ///< 长选项与短选项的对应关系
    std::vector<std::string> positionalArgs_;                  ///< 非选项参数列表
};
