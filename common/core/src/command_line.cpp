#include "vc/core/command_line.h"
#include "vc/core/logging.h"
#include <string>

using namespace std;

void CommandLine::addOption(const std::string &shortName,
                            const std::string &longName,
                            const std::string &description,
                            bool requiresValue,
                            const std::string &defaultValue)
{
    Option opt{longName, description, "", defaultValue, requiresValue, false};
    shortOptions_[shortName] = opt;
    if (!longName.empty())
        longOptions_[longName] = shortName;
}

bool CommandLine::parse(int argc, char *argv[])
{
    if(argc < 2)
        return false;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg.rfind("-", 0) == 0)
        {
            std::string key = arg;
            std::string shortKey;

            // 长选项
            if (arg.rfind("--", 0) == 0)
            {
                key = arg.substr(2);
                if (longOptions_.count(key))
                    shortKey = longOptions_[key];
            }
            // 短选项
            else
            {
                shortKey = arg;
            }

            if (!shortOptions_.count(shortKey))
            {
                VC_WARNING_INFO("未知选项: %s", arg.c_str());
                return false;
            }

            auto &opt = shortOptions_[shortKey];
            opt.isSet = true;

            if (opt.requiresValue)
            {
                if (i + 1 >= argc)
                {
                    VC_WARNING_INFO("选项需要一个值: %s", arg.c_str());
                    return false;
                }
                std::string opt_value(argv[++i]);
                // 去除 opt_value 左右两端的''和""
                if ((opt_value[0] == '\'' && opt_value[opt_value.size() - 1] == '\'') ||
                    (opt_value[0] == '\"' && opt_value[opt_value.size() - 1] == '\"'))
                {
                    opt.value = std::string(opt_value).substr(1, opt_value.size() - 2);
                }
                else
                {
                    opt.value = opt_value;
                }
                opt.value = opt_value;
            }
            else
            {
                opt.value = "true";
            }
        }
        else
        {
            // 非选项参数，可扩展保存到列表
            positionalArgs_.push_back(arg);
        }
    }

    return true;
}

std::string CommandLine::get(const std::string &shortName) const
{
    if (!shortOptions_.count(shortName))
        return "";
    const auto &opt = shortOptions_.at(shortName);
    return opt.isSet ? opt.value : opt.defaultValue;
}

bool CommandLine::isSet(const std::string &shortName) const
{
    if (!shortOptions_.count(shortName))
        return false;
    return shortOptions_.at(shortName).isSet;
}

void CommandLine::printHelp(const std::string &progName) const
{
    std::cout << "Usage: " << progName << " [options]\n";
    for (const auto &[shortName, opt] : shortOptions_)
    {
        std::ostringstream line;
        line << "  " << shortName;
        if (!opt.longName.empty())
            line << ", --" << opt.longName;
        if (opt.requiresValue)
            line << " <value>";
        std::cout << line.str() << "\n      " << opt.description;
        if (!opt.defaultValue.empty())
            std::cout << " (default: " << opt.defaultValue << ")";
        std::cout << "\n";
    }
}

const std::vector<std::string> &CommandLine::getPositionalArgs() const
{
    return positionalArgs_;
}
