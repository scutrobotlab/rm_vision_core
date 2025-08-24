/**
 * @file rune_group.h
 * @brief 神符序列组及数据转换器头文件
 * @author 张峰玮 (3480409161@qq.com)
 * @date 2025-08-24
 */

#pragma once
#include "vc/feature/tracking_feature_node.h"

//! 前向声明数据转换器
class DataConverter;
//! 前向声明神符滤波器
class RuneFilterStrategy;

//! 特征组合体类型定义
using RuneFeatureCombo = std::tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>;
using RuneFeatureComboConst = std::tuple<FeatureNode_cptr, FeatureNode_cptr, FeatureNode_cptr>;

//! 神符序列组
class RuneGroup : public TrackingFeatureNode
{
    using Ptr = std::shared_ptr<RuneGroup>;

    DEFINE_PROPERTY_WITH_INIT(RawDatas, public, protected, (std::deque<double>), {});    //!< 原始角度数据
    DEFINE_PROPERTY_WITH_INIT(AngleSpeeds, public, protected, (std::deque<double>), {}); //!< 角速度数据（顺负逆正）
    DEFINE_PROPERTY(PredictFunc, public, public, (std::function<double(int64_t)>));      //!< 角度变化预测函数
    DEFINE_PROPERTY(RecommendedTicks, public, public, (std::vector<int64_t>));           //!< 推荐击打时间戳列表
    DEFINE_PROPERTY(CamToGyro, public, protected, (PoseNode));                           //!< 相机坐标系到陀螺仪坐标系的变换位姿
    DEFINE_PROPERTY(LastUpdateTick, protected, protected, (int64_t));                    //!< 上一次更新时间戳
    DEFINE_PROPERTY_WITH_INIT(VanishNum, protected, protected, (size_t), 0);             //!< 掉帧计数器
    DEFINE_PROPERTY(LastValidPoseGyro, public, protected, (PoseNode));                   //!< 最后一次有效观测位姿（陀螺仪坐标系）
    std::shared_ptr<RuneFilterStrategy> _filter = nullptr;                               //!< 滤波策略，用于对PNP数据进行滤波

public:
    RuneGroup() = default;

    /**
     * @brief 构建 RuneGroup 智能指针实例
     * @return 返回 RuneGroup 智能指针
     */
    static inline std::shared_ptr<RuneGroup> make_feature() { return std::make_shared<RuneGroup>(); }

    /**
     * @brief 动态类型转换
     * @param[in] p_group 抽象 FeatureNode 指针
     * @return 转换后的 RuneGroup 指针
     */
    static inline std::shared_ptr<RuneGroup> cast(FeatureNode_ptr p_group) { return std::dynamic_pointer_cast<RuneGroup>(p_group); }

    /**
     * @brief 获取所有追踪器
     * @return FeatureNode 智能指针列表
     */
    std::vector<FeatureNode_ptr> getTrackers();

    /**
     * @brief 获取所有追踪器（常量版本）
     * @return FeatureNode 常量智能指针列表
     */
    const std::vector<FeatureNode_cptr> getTrackers() const;

    /**
     * @brief 设置追踪器列表
     * @param[in] trackers 追踪器智能指针列表
     */
    void setTrackers(const std::vector<FeatureNode_ptr> &trackers);

    /**
     * @brief 更新神符序列组状态
     * @param[in] rune_to_cam_raw 相机坐标系下的原始PNP数据
     * @param[in] gyro_data 陀螺仪数据
     * @param[in] tick 当前时间戳
     * @return 更新是否成功
     */
    bool update(const PoseNode &rune_to_cam_raw, const GyroData &gyro_data, int64_t tick);

    /**
     * @brief 利用滤波器获取最新帧的PNP数据
     * @param[out] runeGroup_to_cam 输出相机坐标系PNP数据
     * @return 是否获取成功
     */
    bool getCamPnpDataFromFilter(PoseNode &runeGroup_to_cam) const;

    /**
     * @brief 利用过去的数据预测PNP数据
     * @param[out] runeGroup_to_cam 输出相机坐标系PNP数据
     * @return 是否预测成功
     */
    bool getCamPnpDataFromPast(PoseNode &runeGroup_to_cam) const;

    /**
     * @brief 获取上一帧的特征组合信息
     * @return 上一帧的特征组合常量指针列表
     */
    std::vector<RuneFeatureComboConst> getLastFrameFeatures() const;

    /**
     * @brief 可见性处理
     * @param[in] is_observation 当前帧是否有效观测
     * @return 是否处理失败（失败表示序列组被舍弃）
     */
    bool visibilityProcess(bool is_observation);

    /**
     * @brief 获取当前神符的转角
     * @param[out] angle 输出当前神符转角
     * @return 是否获取成功
     */
    bool getCurrentRotateAngle(float &angle) const;

    /**
     * @brief 神符序列组同步更新
     * @param[in] gyro_data 最新陀螺仪数据
     * @param[in] tick 当前时间戳
     */
    void sync(const GyroData &gyro_data, int64_t tick);

    //! 神符激活信息结构体
    struct ActivationInfo
    {
        int active_target_count = 0;  //!< 当前已激活的靶心个数
        bool has_last_switch = false; //!< 是否存在前一次靶心切换
        int64_t last_switch_tick = 0; //!< 上一次靶心切换时间戳
    };

    /**
     * @brief 获取相机坐标系到陀螺仪坐标系的PNP变换
     * @param[in] gyro_data 陀螺仪数据
     * @return 相机到陀螺仪的位姿
     */
    static PoseNode calcCamToGyroPnpData(const GyroData &gyro_data);

    /**
     * @brief 绘制神符序列组特征
     * @param[in,out] image 待绘制图像
     * @param[in] config 绘制配置（可选）
     */
    virtual void drawFeature(cv::Mat &image, const DrawConfig_cptr &config = nullptr) const override;

    //! 神符中心估计信息
    struct CenterEstimationInfo
    {
        bool is_valid = false;   //!< 是否有效
        cv::Point3d pos_to_gyro; //!< 神符中心陀螺仪坐标系位置
        int64_t tick;            //!< 获取该估计位置的时间戳
    };
    DEFINE_PROPERTY(CenterEstimationInfo, public, protected, (CenterEstimationInfo));

private:
    std::unordered_map<std::string, std::shared_ptr<DataConverter>> __data_converters; //!< 数据转换器映射

    void updatePnpData(const PoseNode &rune_to_gyro, const GyroData &gyro_data);                                        //!< 更新PNP数据
    void clearPreviousData();                                                                                           //!< 清除上一帧数据
    bool correctRoll(const PoseNode &raw_cam_pnp_data, PoseNode &corrected_cam_pnp_data);                               //!< 修正roll角
    PoseNode _correntRoll_last_cam_pnp_data{};                                                                          //!< 上一帧相机PNP数据
    bool _correntRoll_last_cam_pnp_data_flag = false;                                                                   //!< 数据有效标志
    static bool checkExtremePose(const PoseNode &rune_to_cam, const PoseNode &rune_to_gyro, const GyroData &gyro_data); //!< 检测极端位姿
    bool isTargetCenterChanged() const;                                                                                 //!< 是否发生靶心切换
    bool updateCenterEstimation();                                                                                      //!< 更新中心估计信息
};

//! RuneGroup 智能指针类型
using RuneGroup_ptr = std::shared_ptr<RuneGroup>;

#include "vc/feature/rune_data_converter.h"
