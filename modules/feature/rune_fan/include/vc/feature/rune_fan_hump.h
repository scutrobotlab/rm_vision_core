/**
 * @file rune_fan_hump.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 神符扇叶突起点检测与处理头文件
 * @date 2025-08-XX
 *
 *
 * 本文件定义了扇叶突起点相关的检测类和数据结构：
 * - Line: 用于表示直线的基础结构
 * - HumpDetector: 角点查找器，用于检测轮廓中的突起点
 * - RuneFanHump: 突起点基类
 * - TopHump / BottomCenterHump / SideHump: 分别表示顶部、底部中心、侧面突起点
 * - TopHumpCombo: 顶部突起点组合体，用于匹配凸包轮廓
 */

#pragma once

#include "vc/feature/feature_node.h"
#include "opencv2/opencv.hpp"
#include "vc/contour_proc/contour_wrapper.hpp"

/**
 * @brief 表示一条直线的结构体
 *
 * 存储直线的起点和终点索引、角度以及中心点位置，用于轮廓分析与突起点匹配。
 */
struct Line
{
    int start_idx;      //!< 直线起点在轮廓中的索引
    int end_idx;        //!< 直线终点在轮廓中的索引
    float angle;        //!< 直线的角度（单位：度）
    cv::Point2f center; //!< 直线中心点坐标

    /**
     * @brief 构造函数
     * @param[in] _start_idx 起点索引
     * @param[in] _end_idx 终点索引
     * @param[in] _angle 直线角度（单位：度）
     * @param[in] _center 直线中心点坐标
     */
    Line(const int &_start_idx, const int &_end_idx, const float &_angle, const cv::Point2f &_center)
        : start_idx(_start_idx), end_idx(_end_idx), angle(_angle), center(_center) {}
};
/**
 * @brief 角点查找器类
 *
 * 根据轮廓和指定起止索引，计算轮廓段的平均中心点和方向，用于检测突起点。
 */
class HumpDetector
{
private:
    std::vector<cv::Point> points; //!< 轮廓点集
    cv::Point2f direction;         //!< 突起方向（由起点指向终点的单位向量）
    cv::Point2f avePoint;          //!< 轮廓段平均中心点
    int start_idx;                 //!< 起始点索引
    int end_idx;                   //!< 结束点索引

public:
    /**
     * @brief 构造函数
     * @param[in] contour 输入轮廓点集
     * @param[in] _start_idx 起始索引
     * @param[in] _end_idx 结束索引
     */
    HumpDetector(const std::vector<cv::Point> &contour, int _start_idx, int _end_idx);

    //!< 获取突起方向
    inline cv::Point2f getDirection() { return direction; }
    inline cv::Point2f getDirection() const { return direction; }

    //!< 获取平均中心点
    inline cv::Point2f getAvePoint() { return avePoint; }
    inline cv::Point2f getAvePoint() const { return avePoint; }

    //!< 获取轮廓点集
    inline const std::vector<cv::Point> &getPoints() { return points; }
    inline const std::vector<cv::Point> &getPoints() const { return points; }

    //!< 获取起始索引
    inline int getStartIdx() { return start_idx; }
    inline int getStartIdx() const { return start_idx; }

    //!< 获取结束索引
    inline int getEndIdx() { return end_idx; }
    inline int getEndIdx() const { return end_idx; }

    //!< 获取起始点坐标
    inline cv::Point2f getStartPoint() { return points.front(); }
    inline cv::Point2f getStartPoint() const { return points.front(); }

    //!< 获取结束点坐标
    inline cv::Point2f getEndPoint() { return points.back(); }
    inline cv::Point2f getEndPoint() const { return points.back(); }

    /**
     * @brief 判断三点是否共线
     * @param[in] p1 第一个点
     * @param[in] p2 第二个点
     * @param[in] p3 第三个点
     * @return 返回三点的角度偏差量（单位：度）
     */
    static double CheckCollinearity(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &p3);

    /**
     * @brief 判断三条射线是否同向
     * @param[in] direction_1 射线1方向
     * @param[in] direction_2 射线2方向
     * @param[in] direction_3 射线3方向
     * @return 返回方向偏差量（单位：度）
     */
    static double CheckAlignment(const cv::Point2f &direction_1, const cv::Point2f &direction_2, const cv::Point2f &direction_3);
};

/**
 * @brief 突起点基类
 *
 * 存储突起点的中心点、顶点和方向信息。
 * 提供可视化轮廓接口，用于调试和凸包匹配。
 */
class RuneFanHump
{
protected:
    cv::Point2f center{};    //!< 突起中心点
    cv::Point2f vertex{};    //!< 突起顶点
    cv::Point2f direction{}; //!< 突起方向（从中心指向顶点）

public:
    virtual ~RuneFanHump() = 0; //!< 虚析构函数，确保派生类正确析构

    //!< 获取中心点
    inline cv::Point2f getCenter() { return center; }
    inline cv::Point2f getCenter() const { return center; }

    //!< 获取顶点
    inline cv::Point2f getVertex() { return vertex; }
    inline cv::Point2f getVertex() const { return vertex; }

    //!< 获取方向
    inline cv::Point2f getDirection() { return direction; }
    inline cv::Point2f getDirection() const { return direction; }

    /**
     * @brief 获取调试用的可视化轮廓
     *
     * 根据中心点、顶点和方向生成简化轮廓，用于可视化或凸包计算。
     * @return 返回突起点对应的轮廓对象（Contour_cptr）
     */
    virtual Contour_cptr getContour() const
    {
        if (__contour_ptr == nullptr)
        {
            std::vector<cv::Point> contour;
            contour.push_back(center);
            contour.push_back(vertex);
            contour.push_back(center + direction * 20);
            __contour_ptr = ContourWrapper<int>::make_contour(contour);
        }
        return __contour_ptr;
    }

private:
    mutable Contour_ptr __contour_ptr = nullptr; //!< 调试用轮廓缓存
};

inline RuneFanHump::~RuneFanHump() = default;

struct TopHumpCombo;

/**
 * @brief 顶部突起类
 *
 * 表示轮廓的顶部突起信息，包括上升点和下降点的索引及迭代状态。
 * 可用于查找和过滤轮廓的顶部突起点。
 */
class TopHump : public RuneFanHump
{
private:
    enum state
    {
        UP,
        DOWN
    }; //!< 上升/下降状态

    int up_start_idx = -1;   //!< 上升点起始下标
    int down_start_idx = -1; //!< 下降点起始下标
    int up_end_idx = -1;     //!< 上升点结束下标
    int down_end_idx = -1;   //!< 下降点结束下标
    int line_pair_idx = -1;  //!< 线段组下标

    int up_itertaion_num = 0;   //!< 上升点迭代次数
    int down_itertaion_num = 0; //!< 下降点迭代次数
    int end_iteration_up_idx;   //!< 最后一次迭代上升点下标
    state current_state = UP;   //!< 当前状态，默认为上升

public:
    TopHump() = default;

    /**
     * @brief 构造函数
     * @param[in] up_start_idx 上升点起始下标
     * @param[in] up_end_idx 上升点结束下标
     * @param[in] down_start_idx 下降点起始下标
     * @param[in] down_end_idx 下降点结束下标
     * @param[in] _line_pair_idx 线段组下标
     * @param[in] _direction 突起方向
     * @param[in] _center 突起中心点
     */
    TopHump(int _up_start_idx, int _up_end_idx, int _down_start_idx, int _down_end_idx, int _line_pair_idx, const cv::Point2f &_direction, const cv::Point2f &_center);

    /**
     * @brief 根据上升点和下降点下标构造突起点
     * @param[in] up_idx 上升点下标
     * @param[in] down_idx 下降点下标
     * @param[in] find_state 查找状态（上升或下降）
     * @param[in] _direction 突起方向
     * @return 是否构造成功
     */
    TopHump(int up_idx, int down_idx, state find_state, const cv::Point2f &_direction);

    /**
     * @brief 获取顶部突起点数组
     * @param[in] contour_plus 加长轮廓
     * @param[in] contour_center 原轮廓中心
     * @param[in,out] line_pairs 待匹配直线对
     * @return 成功返回突起点数组，否则返回空数组
     */
    static std::vector<TopHump> getTopHumps(const std::vector<cv::Point> &contour_plus, const cv::Point2f &contour_center, const std::vector<std::tuple<Line, Line>> &line_pairs);

    /**
     * @brief 获取顶部突起点组合体数组
     * @param[in] contours 轮廓集合
     * @param[in] contour_centers 各轮廓中心点
     * @param[in] line_pairs 待匹配直线对
     * @return 顶部突起组合体数组
     */
    static std::vector<TopHumpCombo> getTopHumpCombos(const std::vector<Contour_cptr> &contours, const std::vector<cv::Point2f> &contour_centers, const std::vector<std::tuple<Line, Line>> &line_pairs);

    //!< 获取线段组下标
    inline int getLinePairIdx() { return line_pair_idx; }
    inline int getLinePairIdx() const { return line_pair_idx; }

    static bool make_TopHumps(TopHumpCombo &hump_combo, const cv::Point2f &contour_center, double &delta);
    static bool setVertex(TopHump &hump, const std::vector<cv::Point> &contour_plus);

    /**
     * @brief 过滤异常突起点
     * @param[in] contour_plus 加长轮廓
     * @param[in,out] humps 输入突起点集，输出过滤后的突起点集
     * @return 是否过滤成功
     */
    static bool filter(const std::vector<cv::Point> &contour_plus, std::vector<TopHump> &humps);

private:
    void update(const TopHump &hump);
    static bool getAllHumps2(const std::vector<cv::Point> &contours_plus, const std::vector<std::tuple<Line, Line>> &line_pairs, std::vector<TopHump> &humps);
};

/**
 * @brief 顶部突起组合体
 *
 * 由三个顶部突起点组成的组合，用于凸包匹配和可视化。
 */
struct TopHumpCombo
{
    TopHumpCombo(const TopHump &hump_1, const TopHump &hump_2, const TopHump &hump_3)
    {
        __humps[0] = hump_1;
        __humps[1] = hump_2;
        __humps[2] = hump_3;
    }

    /**
     * @brief 获取组合体凸包轮廓（调试用）
     * @return 返回组合体凸包轮廓
     */
    Contour_cptr getContour() const
    {
        if (__contour_ptr == nullptr)
        {
            __contour_ptr = ContourWrapper<int>::getConvexHull({__humps[0].getContour(), __humps[1].getContour(), __humps[2].getContour()});
        }
        return __contour_ptr;
    }

    std::tuple<TopHump, TopHump, TopHump> getHumpsTuple() const { return std::make_tuple(__humps[0], __humps[1], __humps[2]); }
    std::vector<TopHump> getHumpsVector() { return {__humps[0], __humps[1], __humps[2]}; }
    std::array<TopHump, 3> &humps() { return __humps; }

private:
    std::array<TopHump, 3> __humps;    //!< 三个突起点
    mutable Contour_ptr __contour_ptr; //!< 调试用轮廓缓存
};

/**
 * @brief 底部中心突起类
 *
 * 表示轮廓的底部中心突起信息。
 */
class BottomCenterHump : public RuneFanHump
{
private:
    int idx = 0; //!< 突起点下标（在原轮廓中）

public:
    BottomCenterHump() = default;

    /**
     * @brief 构造函数
     * @param[in] _center 突起中心点
     * @param[in] direction 突起方向
     * @param[in] _idx 突起点下标
     */
    BottomCenterHump(const cv::Point2f &_center, const cv::Point2f &direction, const int _idx);

    /**
     * @brief 获取底部中心突起点数组
     * @param[in] contour 原轮廓
     * @param[in] contour_center 原轮廓中心
     * @param[in] top_humps 顶部突起点数组
     * @return 成功返回突起点数组，否则返回空数组
     */
    static std::vector<BottomCenterHump> getBottomCenterHump(const std::vector<cv::Point> &contour, const cv::Point2f &contour_center, const std::vector<TopHump> &top_humps);

private:
    static bool getAllHumps(const std::vector<cv::Point> &contour, const std::vector<TopHump> &top_humps, std::vector<BottomCenterHump> &humps);
    static bool filter(const std::vector<cv::Point> &contour, const std::vector<TopHump> &top_humps, std::vector<BottomCenterHump> &humps);
};

/**
 * @brief 侧面突起类
 *
 * 表示轮廓的侧面突起信息。
 */
class SideHump : public RuneFanHump
{
private:
    int up_start_idx = -1;   //!< 上升点起始下标
    int down_start_idx = -1; //!< 下降点起始下标
    int up_end_idx = -1;     //!< 上升点结束下标
    int down_end_idx = -1;   //!< 下降点结束下标

public:
    SideHump() = default;
    SideHump(const cv::Point2f &_direction, const cv::Point2f &_center);

    /**
     * @brief 获取所有侧面突起点
     * @param[in] contour_plus 加长轮廓
     * @param[in] contour_center 轮廓中心
     * @param[in] top_humps 顶部突起点
     * @param[in] bottom_center_humps 底部中心突起点
     * @param[in,out] line_pairs 待识别直线对
     * @return 所有侧面突起点
     */
    static std::vector<SideHump> getSideHumps(const std::vector<cv::Point> &contour_plus, const cv::Point2f &contour_center, const std::vector<TopHump> &top_humps, const std::vector<BottomCenterHump> &bottom_center_humps, std::vector<std::tuple<Line, Line>> &line_pairs);

private:
    static bool getAllHumps(const std::vector<TopHump> &top_humps, const std::vector<std::tuple<Line, Line>> &line_pairs, std::vector<SideHump> &humps);
    static bool setVertex(SideHump &hump, const std::vector<cv::Point> &contour_plus);
    static bool filter(std::vector<SideHump> &humps);
    static bool make_SideHumps(std::vector<SideHump> &humps, const std::vector<TopHump> &top_humps, const cv::Point2f &contour_center, double &delta);
};
