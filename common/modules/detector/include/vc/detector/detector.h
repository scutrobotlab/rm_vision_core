/**
 * @file detector.h
 * @author zhaoxi (535394140@qq.com)
 * @brief 抽象识别类头文件
 * @date 2025-08-20
 */

#pragma once

#include "vc/core/yml_manager.hpp"
#include "vc/feature/feature_node.h"

struct DetectorInput;
struct DetectorOutput;

/**
 * @brief 识别器基类
 *
 * @note 提供识别器的通用接口定义，所有具体识别器需继承并实现
 *       `detect` 方法以完成自定义的识别逻辑。
 */
class Detector
{
public:
    /// @brief 默认析构函数
    virtual ~Detector() = default;

    /**
     * @brief 识别器主函数
     *
     * @param[in] input 识别器输入参数
     * @param[out] output 识别器输出结果
     *
     * @note 该函数为纯虚函数，需由具体识别器实现。
     */
    virtual void detect(DetectorInput &input, DetectorOutput &output) = 0;
};

/// @brief 识别器基类智能指针类型
using Detector_ptr = std::shared_ptr<Detector>;

#include "detector_input.h"  ///< DetectorInput 结构体定义
#include "detector_output.h" ///< DetectorOutput 结构体定义
