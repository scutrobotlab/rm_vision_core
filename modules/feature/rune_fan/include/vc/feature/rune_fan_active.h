/**
 * @file rune_fan_active.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 已激活的神符扇叶特征类头文件
 * @date 2025-7-15
 * @details 定义了继承自 RuneFan 的已激活扇叶特征类 RuneFanActive，
 *          包含构造、残缺扇叶处理、PNP位姿构造及可视化接口。
 *          提供静态方法用于批量查找、构造和获取扇叶角点。
 */

#pragma once

#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_fan_hump.h"

/**
 * @class RuneFanActive
 * @brief 已激活的神符扇叶特征类
 * @details 继承自 RuneFan，包含完整扇叶角点信息以及残缺扇叶处理接口。
 *          提供构造、查找、PNP构造及绘制特征接口。
 */
class RuneFanActive : public RuneFan
{
    using Ptr = std::shared_ptr<RuneFanActive>; //!< 智能指针类型定义

    DEFINE_PROPERTY(RotatedRect, public, public, (cv::RotatedRect));                            //!< 扇叶最小外接矩形
    DEFINE_PROPERTY(TopHumpCorners, protected, protected, (std::vector<cv::Point2f>));          //!< 扇叶顶部突起角点
    DEFINE_PROPERTY(BottomCenterHumpCorners, protected, protected, (std::vector<cv::Point2f>)); //!< 扇叶底部中心突起角点
    DEFINE_PROPERTY(SideHumpCorners, protected, protected, (std::vector<cv::Point2f>));         //!< 扇叶侧面突起角点
    DEFINE_PROPERTY(BottomSideHumpCorners, protected, protected, (std::vector<cv::Point2f>));   //!< 扇叶底部侧面突起角点

public:
    RuneFanActive() = default;                     //!< 默认构造函数
    RuneFanActive(const RuneFanActive &) = delete; //!< 禁用拷贝构造
    RuneFanActive(RuneFanActive &&) = delete;      //!< 禁用移动构造
    virtual ~RuneFanActive() = default;            //!< 默认析构

    /**
     * @brief 构造已激活的 RuneFan 对象
     * @param contour 扇叶轮廓
     * @param rotated_rect 扇叶最小外接矩形
     * @param top_hump_corners 顶部突起角点
     * @param bottom_center_hump_corners 底部中心突起角点
     * @param side_hump_corners 侧面突起角点
     * @param bottom_side_hump_corners 底部侧面突起角点
     */
    RuneFanActive(const Contour_cptr &contour, const cv::RotatedRect &rotated_rect, const std::vector<cv::Point2f> &top_hump_corners, const std::vector<cv::Point2f> &bottom_center_hump_corners, const std::vector<cv::Point2f> &side_hump_corners, const std::vector<cv::Point2f> &bottom_side_hump_corners);

    /**
     * @brief 构造已激活的 RuneFan 对象（多轮廓版）
     * @param contours 扇叶轮廓点集
     * @param top_hump_corners 顶部突起角点
     * @param direction 扇叶方向
     */
    RuneFanActive(const std::vector<Contour_cptr> &contours, const std::vector<cv::Point2f> &top_hump_corners, const cv::Point2f &direction);

    /**
     * @brief 将通用 FeatureNode 智能指针转换为 RuneFanActive 智能指针
     * @param p_feature 输入的 FeatureNode 智能指针
     * @return 转换后的 RuneFanActive 智能指针
     */
    static inline std::shared_ptr<RuneFanActive> cast(FeatureNode_ptr p_feature) { return std::dynamic_pointer_cast<RuneFanActive>(p_feature); }

    /**
     * @brief 找到所有已激活扇叶
     * @param[out] fans 返回找到的神符扇叶
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     */
    static void find(std::vector<FeatureNode_ptr> &fans, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 找到所有残缺的已激活扇叶
     * @param[out] fans 返回找到的残缺的已激活扇叶
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可跳过构造的轮廓下标集合
     * @param[in] rotate_center 旋转中心
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     * @return 是否成功找到残缺扇叶
     */
    static bool find_incomplete(std::vector<FeatureNode_ptr> &fans, const std::vector<Contour_cptr> &contours, const std::vector<cv::Vec4i> &hierarchy, const std::unordered_set<size_t> &mask, const cv::Point2f &rotate_center, std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 已激活 RuneFan 的强制构造接口（角点输入）
     * @param[in] top_corners 顶部突起角点
     * @param[in] bottom_center_corners 底部中心突起角点
     * @param[in] side_corners 侧面突起角点
     * @param[in] bottom_side_corners 底部侧面突起角点
     * @return 成功返回 RuneFanActive 智能指针，否则返回 nullptr
     * @note 利用PNP解算获取角点构造，输入角点顺序有要求
     */
    static Ptr make_feature(const std::vector<cv::Point2d> &top_corners, const std::vector<cv::Point2d> &bottom_center_corners, const std::vector<cv::Point2d> &side_corners, const std::vector<cv::Point2d> &bottom_side_corners);

protected:
    /**
     * @brief 已激活 RuneFan 的轮廓构造接口
     * @param[in] contour 轮廓
     * @return 成功返回 RuneFanActive 智能指针，否则返回 nullptr
     */
    static Ptr make_feature(const Contour_cptr &contour);

    /**
     * @brief 已激活 RuneFan 的缺陷构造接口（单独角点）
     * @param[in] hump_1 第一个突起点
     * @param[in] hump_2 第二个突起点
     * @param[in] hump_3 第三个突起点
     * @return 成功返回 RuneFanActive 智能指针，否则返回 nullptr
     */
    static Ptr make_feature(const std::tuple<TopHump, Contour_cptr> &hump_1, const std::tuple<TopHump, Contour_cptr> &hump_2, const std::tuple<TopHump, Contour_cptr> &hump_3);

    /**
     * @brief 通过扇叶位姿PNP解算结果构造 RuneFan
     * @param[in] fan_to_cam 扇叶相对于相机的位姿
     * @param[in] is_active 是否激活
     * @return 构造成功返回智能指针，否则返回 nullptr
     */
    static Ptr make_feature(const PoseNode &fan_to_cam, bool is_active);

    /**
     * @brief 绘制特征
     * @param image 要绘制的图像
     * @param config 绘制配置，可选
     * @note 默认实现为空，子类可重载实现具体绘制逻辑
     */
    virtual void drawFeature(cv::Mat &image, const DrawConfig_cptr &config = nullptr) const override;

public:
    /**
     * @brief 获取轮廓角度数组
     * @param[in] contour_plus 加长后的轮廓
     * @return 轮廓角度矩阵
     */
    static cv::Mat getAngles(const std::vector<cv::Point> &contour_plus);

    /**
     * @brief 获取角度矩阵梯度
     * @param[in] angles_mat 角度矩阵
     * @return 梯度矩阵
     */
    static cv::Mat getGradient(const cv::Mat &angles_mat);

    /**
     * @brief 获取所有直线对
     * @param[in] contour_plus 加长后的轮廓
     * @param[in] angles_mat 角度矩阵
     * @param[in] gradient_mat 梯度矩阵
     * @param[out] line_pairs 输出的直线对
     * @return 是否成功获取直线对
     */
    static bool getLinePairs(const std::vector<cv::Point> &contour_plus, const cv::Mat &angles_mat, const cv::Mat &gradient_mat, std::vector<std::tuple<Line, Line>> &line_pairs);

    /**
     * @brief 获取角点在图像坐标系和特征坐标系下的坐标
     * @return [0] 图像坐标系 [1] 特征坐标系 [2] 各点权重
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> override;

    /**
     * @brief 获取已激活扇叶的所有角点
     * @param[in] contour 扇叶轮廓
     * @param[out] top_hump_corners 输出顶部角点
     * @param[out] bottom_center_hump_corners 输出底部中心角点
     * @param[out] side_hump_corners 输出侧面角点
     * @param[out] bottom_side_hump_corners 输出底部侧面角点
     * @param[out] direction 输出扇叶方向单位向量（指向神符中心）
     * @return 是否成功获取角点
     */
    static bool getActiveFunCorners(const Contour_cptr &contour, std::vector<cv::Point2f> &top_hump_corners, std::vector<cv::Point2f> &bottom_center_hump_corners, std::vector<cv::Point2f> &side_hump_corners, std::vector<cv::Point2f> &bottom_side_hump_corners);
};

using RuneFanActive_ptr = std::shared_ptr<RuneFanActive>;        //!< 可修改的智能指针
using RuneFanActive_cptr = std::shared_ptr<const RuneFanActive>; //!< 只读智能指针
