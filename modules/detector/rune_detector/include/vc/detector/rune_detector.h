#pragma once

#include "vc/detector/detector.h"

//! 特征组合体
using RuneFeatureCombo = std::tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>;
using RuneFeatureComboConst = std::tuple<FeatureNode_cptr, FeatureNode_cptr, FeatureNode_cptr>;

class RuneDetector : public Detector
{
public:
    RuneDetector() = default;

    //! 最新帧的输入图像
    DEFINE_PROPERTY(InputImage, public, protected, (cv::Mat));
    //! 时间戳
    DEFINE_PROPERTY(Tick, public, protected, (int64_t));
    //! 陀螺仪数据
    DEFINE_PROPERTY(GyroData, public, protected, (GyroData));

public:
    /**
     * @brief 识别器的主函数
     * @param input 识别器输入
     * @param output 识别器输出
     */
    void detect(DetectorInput &input, DetectorOutput &output) override;

    //! 构建 RuneDetector
    static inline std::unique_ptr<RuneDetector> make_detector() { return std::make_unique<RuneDetector>(); }

private:
    /**
     * @brief 二值化操作
     *
     * @param[in] src 输入原图
     * @param[out] bin 输出二值化图像
     * @param[in] target_color 目标颜色
     * @param[in] threshold 阈值
     */
    static void binary(const cv::Mat &src, cv::Mat &bin, PixChannel target_color, uint8_t threshold);

    /**
     * @brief 找出所有的配对神符特征
     *
     * @param[in] src 预处理之后的图像
     * @param[in] center_estimation_info 神符中心的估计位置
     * @param[out] features 找到的所有特征
     * @param[out] matched_features 配对好的特征 (靶心、中心、扇叶),未找出的特征将会用 nullptr 补齐
     */
    static bool findFeatures(cv::Mat src, std::vector<FeatureNode_cptr> &features, std::vector<RuneFeatureCombo> &matched_features);

    /**
     * @brief 初步获取所有的神符特征
     * @param[out] targets_inactive 未激活的靶心
     * @param[out] targets_active 激活的靶心
     * @param[out] fans_inactive 未激活的扇叶
     * @param[out] fans_active 激活的扇叶
     * @param[out] centers 神符中心
     * @param[in] contours 输入的轮廓
     * @param[in] hierarchy 轮廓的层级关系
     * @param[in] continue_idx 需要跳过的轮廓索引
     */
    static void extractRuneFeatures(std::vector<FeatureNode_ptr> &targets_inactive,std::vector<FeatureNode_ptr> &targets_active,std::vector<FeatureNode_ptr> &fans_inactive,std::vector<FeatureNode_ptr> &fans_active,std::vector<FeatureNode_ptr> &centers,const std::vector<Contour_cptr> &contours,const std::vector<cv::Vec4i> &hierarchy,std::unordered_set<size_t> &continue_idx);

    /**
     * @brief 将神符组合体与追踪器进行匹配
     *
     * @param[in] combos 所有的神符组合体 [长度为5]
     * @param[in] trackers 所有的追踪器 [长度为5]
     * @param[in] is_vanish_update 是否为掉帧更新
     *
     * @return bool 是否匹配成功 [ 要求: 神符组合体数量 = 追踪器数量]
     */
    bool match(const std::vector<FeatureNode_ptr> &combos, std::vector<FeatureNode_ptr> &trackers, bool is_vanish_update);

    /**
     * @brief 筛选合适的未激活的靶心
     *
     * @param[in,out] inactive_targets 需要筛选的未激活的靶心
     */
    static bool filterInactiveTarget(std::vector<FeatureNode_ptr> &inactive_targets);

    /**
     * @brief 筛选合适的激活的靶心
     *
     * @param[in,out] active_targets 需要筛选的激活的靶心
     * @param[in] inactive_targets 未激活的靶心
     */
    static bool filterActiveTarget(std::vector<FeatureNode_ptr> &active_targets, std::vector<FeatureNode_ptr> &inactive_targets);
    /**
     * @brief 筛选合适的未激活的扇叶
     *
     * @param[in,out] inactive_fans 需要筛选的未激活的扇叶
     * @param[in] inactive_targets 未激活的靶心
     * @param[in] active_targets 激活的靶心
     * @param[in] active_fans 激活的扇叶
     */
    static bool filterInactiveFan(std::vector<FeatureNode_ptr> &inactive_fans,
                                  const std::vector<FeatureNode_ptr> &inactive_targets,
                                  const std::vector<FeatureNode_ptr> &active_targets,
                                  const std::vector<FeatureNode_ptr> &active_fans);

    /**
     * @brief 筛选合适的神符中心
     *
     * @param[in,out] centers 需要筛选的神符中心
     * @param[in] inactive_targets 未激活的靶心
     * @param[in] active_targets 激活的靶心
     * @param[in] inactive_fans 未激活的扇叶
     * @param[in] active_fans 激活的扇叶
     */
    static bool filterCenter(std::vector<FeatureNode_ptr> &centers,
                             const std::vector<FeatureNode_ptr> &inactive_targets,
                             const std::vector<FeatureNode_ptr> &active_targets,
                             const std::vector<FeatureNode_ptr> &inactive_fans,
                             const std::vector<FeatureNode_ptr> &active_fans);

    /**
     * @brief 筛选残缺构造的已激活扇叶
     *
     * @param[in,out] fans_Incomplete 需要筛选的残缺构造的已激活扇叶
     * @param[in] inactive_targets 未激活的靶心
     * @param[in] active_targets 激活的靶心
     * @param[in] inactive_fans 未激活的扇叶
     * @param[in] active_fans 激活的扇叶
     * @param[in] best_center 最佳神符中心点
     */
    static bool filterActiveFanIncomplete(std::vector<FeatureNode_ptr> &fans_Incomplete,
                                          const std::vector<FeatureNode_ptr> &inactive_targets,
                                          const std::vector<FeatureNode_ptr> &active_targets,
                                          const std::vector<FeatureNode_ptr> &inactive_fans,
                                          const std::vector<FeatureNode_ptr> &active_fans,
                                          const FeatureNode_ptr &best_center);
    /**
     * @brief 过滤距离过远的轮廓
     *
     * @param[in] contours 所有轮廓
     * @param[in] targets 所有神符靶心
     * @param[in] centers 所有神符中心点
     * @param[in] fans 所有神符扇叶
     * @param[out] far_contour_idxs 距离过远的轮廓下标
     */
    static bool filterFarContours(const std::vector<Contour_cptr> &contours,
                                  const std::vector<FeatureNode_ptr> &targets,
                                  const FeatureNode_ptr &center,
                                  const std::vector<FeatureNode_ptr> &fans,
                                  const std::unordered_set<size_t> &mask,
                                  std::unordered_set<size_t> &far_contour_idxs);

    /**
     * @brief 获取旋转中心
     *
     * @param[in] targets 所有神符靶心
     * @param[in] fans 所有神符扇叶
     * @param[out] rotate_center 旋转中心
     * @param[in]  is_accurate 是否精确计算(默认为true)
     *
     * @return 非精确状态下，求出的旋转中心只是近似值。但是会提高成功率
     *
     */
    static bool getRotateCenter(const std::vector<FeatureNode_ptr> &targets, const std::vector<FeatureNode_ptr> &fans, cv::Point2f &rotate_center);

    /**
     * @brief 获取最佳的神符中心点
     * @note 在已知的所有中心点中寻找到最适合作为中心点的一个特征
     *
     * @param[in] p_targets 所有神符靶心
     * @param[in] p_centers 所有神符中心点
     * @return RuneC_ptr
     */
    FeatureNode_ptr getBestCenter(const std::vector<FeatureNode_ptr> &p_targets, const std::vector<FeatureNode_ptr> &p_centers);

    /**
     * @brief 获取配对后的特征
     *
     * @param[in] p_targets 所有神符靶心
     * @param[in] p_center 最佳神符中心点
     * @param[in] p_fans 所有神符扇叶
     * @return 配对后的特征组
     */
    static std::vector<RuneFeatureCombo> getMatchedFeature(const std::vector<FeatureNode_ptr> &p_targets, const FeatureNode_ptr &p_center, const std::vector<FeatureNode_ptr> &p_fans);

    /**
     * @brief 获取神符在相机坐标系下的 PNP 解算数据
     *
     * @param[out] camera_pnp_data PNP解算数据
     * @param[in] group 神符组
     * @param[in] matched_features 配对好的特征 (靶心、中心、扇叶)
     * @param[in] gyro_data 陀螺仪数据
     */
    bool getPnpData(PoseNode &camera_pnp_data,
                    const FeatureNode_ptr &group,
                    const std::vector<RuneFeatureComboConst> &matched_features) const;

    /**
     * @brief 获取所有神符组合体
     *
     * @param[out] runes 所有神符组合体
     * @param[in] group 神符组
     * @param[in] matched_features 配对好的特征 (靶心、中心、扇叶)
     * @param[in] camera_pnp_data 神符序列组在相机坐标系下的 PNP 解算数据
     *
     * @note    1. `matched_features` 用于提供神符类型信息
     *          2. `camera_pnp_data` 用于提供神符在相机坐标系下的位姿信息
     */
    bool getRunes(std::vector<FeatureNode_ptr> &current_combos,
                  const FeatureNode_ptr &group,
                  const std::vector<RuneFeatureComboConst> &matched_features,
                  const PoseNode &camera_pnp_data) const;

    /**
     * @brief 获取神符组在相机坐标系下的PNP数据
     *
     * @param group 神符组
     * @param points_2d 2D点
     * @param points_3d 3D点
     * @param point_weights 点的权重
     *
     * @return PnP解算的数据
     */
    bool getCameraPnpData(const FeatureNode_ptr &group,
                          const std::vector<cv::Point2f> &points_2d,
                          const std::vector<cv::Point3f> &points_3d,
                          const std::vector<float> &point_weights,
                          PoseNode &pnp_data) const;

};
