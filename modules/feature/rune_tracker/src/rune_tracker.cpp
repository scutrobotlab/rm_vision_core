#include "vc/feature/rune_tracker.h"
#include "vc/feature/rune_tracker_param.h"
#include "vc/feature/rune_combo.h"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;

void RuneTracker::updateFromRune(FeatureNode_ptr p_combo)
{
    setImageCache(p_combo->getImageCache());
    setPoseCache(p_combo->getPoseCache());
    getHistoryNodes().push_front(p_combo);
    getHistoryTicks().push_front(p_combo->getTick());
}

void RuneTracker::update(FeatureNode_ptr p_rune, int64 tick, const GyroData &gyro_data)
{
    // 数据更新
    updateFromRune(p_rune);
    auto &__history_deque = getHistoryNodes();
    auto &__tick_deque = getHistoryTicks();

    if (__history_deque.size() >= static_cast<size_t>(rune_tracker_param.MAX_DEQUE_SIZE))
        __history_deque.pop_back();
    if (__tick_deque.size() >= static_cast<size_t>(rune_tracker_param.MAX_DEQUE_SIZE))
        __tick_deque.pop_back();
}

void RuneTracker::updateVisible(bool is_visible)
{
    auto &__vanish_num = getDropFrameCount();
    if (is_visible)
    {
        __vanish_num = 0;
    }
    else
    {
        ++__vanish_num;
    }
}

inline void drawPentagonWedge(cv::Mat &img, const PoseNode &p,
                              float radius, float height,
                              int sector_index = 0,
                              cv::Point2f center_offset = cv::Point2f(0.0f, 0.0f),
                              cv::Scalar c = cv::Scalar(0, 255, 0))
{
    if (radius <= 0.0f || height == 0.0f)
        return;

    float hz = height / 2.0f; // 上下各半高
    const float angle_step = 2.0f * CV_PI / 5.0f;

    // 规范化 sector_index
    int i0 = ((sector_index % 5) + 5) % 5;

    // 当前扇区的两个顶点角度（保持右手系）
    float angle1 = -(i0 * angle_step - CV_PI / 2.0f);
    float angle2 = -(((i0 + 1) % 5) * angle_step - CV_PI / 2.0f);

    // --- 构造 3D 顶点 ---
    std::vector<cv::Point3f> pts3d;
    // 下底（三角形 z = -hz）
    pts3d.emplace_back(center_offset.x, center_offset.y, -hz); // 0: 底中心
    pts3d.emplace_back(center_offset.x + radius * std::cos(angle1),
                       center_offset.y + radius * std::sin(angle1), -hz); // 1: 底顶点1
    pts3d.emplace_back(center_offset.x + radius * std::cos(angle2),
                       center_offset.y + radius * std::sin(angle2), -hz); // 2: 底顶点2
    // 上底（三角形 z = +hz）
    pts3d.emplace_back(center_offset.x, center_offset.y, hz); // 3: 顶中心
    pts3d.emplace_back(center_offset.x + radius * std::cos(angle1),
                       center_offset.y + radius * std::sin(angle1), hz); // 4: 顶顶点1
    pts3d.emplace_back(center_offset.x + radius * std::cos(angle2),
                       center_offset.y + radius * std::sin(angle2), hz); // 5: 顶顶点2

    // --- 投影到图像平面 ---
    std::vector<cv::Point2f> pts2d;
    projectPoints(pts3d, p.rvec(), p.tvec(),
                  camera_param.cameraMatrix, camera_param.distCoeff, pts2d);

    // --- 绘制线条 ---
    // 下底
    cv::line(img, pts2d[0], pts2d[1], c, 1);
    cv::line(img, pts2d[1], pts2d[2], c, 1);
    cv::line(img, pts2d[2], pts2d[0], c, 1);

    // 上底
    cv::line(img, pts2d[3], pts2d[4], c, 1);
    cv::line(img, pts2d[4], pts2d[5], c, 1);
    cv::line(img, pts2d[5], pts2d[3], c, 1);

    // 竖直连线
    cv::line(img, pts2d[0], pts2d[3], c, 1);
    cv::line(img, pts2d[1], pts2d[4], c, 1);
    cv::line(img, pts2d[2], pts2d[5], c, 1);
}

inline void drawCube(cv::Mat &img, const PoseNode &p, float x_len, float y_len, float z_len, cv::Scalar c)
{
    float hx = x_len / 2, hy = y_len / 2, hz = z_len / 2;
    vector<Point3f> pts3d = {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz}, {-hx, hy, -hz}, {-hx, -hy, hz}, {hx, -hy, hz}, {hx, hy, hz}, {-hx, hy, hz}};
    vector<Point2f> pts2d;
    projectPoints(pts3d, p.rvec(), p.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, pts2d);
    for (int i = 0; i < 4; i++)
    {
        line(img, pts2d[i], pts2d[(i + 1) % 4], c, 1);
        line(img, pts2d[i + 4], pts2d[(i + 1) % 4 + 4], c, 1);
        line(img, pts2d[i], pts2d[i + 4], c, 1);
    }
}

void RuneTracker::drawFeature(cv::Mat &image, const DrawConfig_cptr &config) const
{
    auto &pose_info = getPoseCache();
    do
    {
        if (pose_info.getPoseNodes().count(CoordFrame::CAMERA) == 0)
            break;
        auto &p = pose_info.getPoseNodes().at(CoordFrame::CAMERA);
        Scalar color = Scalar(0, 255, 0);
        auto type = RuneCombo::cast(this->getHistoryNodes().front())->getRuneType();
        if(type == RuneType::STRUCK)
            color = Scalar(0, 255, 0);
        else if(type == RuneType::PENDING_STRUCK)
            color = Scalar(0, 255, 255);
        else if(type == RuneType::UNKNOWN || type == RuneType::UNSTRUCK)
            color = Scalar(255, 255, 255);
        // drawCube(image, p, 500, 500, 300, color);
    } while (0);
}
