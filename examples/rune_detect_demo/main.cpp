#include <opencv2/opencv.hpp>
#include "rune_detect_demo/rune_detect_demo.h"
#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_target_active.h"
#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_active.h"
#include "vc/feature/rune_fan_inactive.h"
#include "vc/feature/rune_center.h"
#include "vc/core/debug_tools/window_auto_layout.h"
#include "vc/core/debug_tools/param_view_manager.h"
#include "vc/core/yml_manager.hpp"
#include "vc/core/debug_tools.h"
#include "vc/feature/rune_fan.h"
#include "vc/detector/rune_detector.h"
#include "vc/feature/rune_group.h"
#include "vc/feature/rune_combo.h"
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

inline cv::Mat binary(cv::Mat src, PixChannel ch1, PixChannel ch2, uint8_t thresh)
{
    if (src.type() != CV_8UC3)
    {
        VC_THROW_ERROR("The image type of \"src\" is incorrect");
    }
    Mat bin = Mat::zeros(Size(src.cols, src.rows), CV_8UC1);
    // Image process
    parallel_for_(Range(0, src.rows),
                  [&](const Range &range)
                  {
                      uchar *data_src = nullptr;
                      uchar *data_bin = nullptr;
                      for (int row = range.start; row < range.end; ++row)
                      {
                          data_src = src.ptr<uchar>(row);
                          data_bin = bin.ptr<uchar>(row);
                          for (int col = 0; col < src.cols; ++col)
                              if (data_src[3 * col + ch1] - data_src[3 * col + ch2] > thresh)
                                  data_bin[col] = 255;
                      }
                  });
    return bin;
}

int main(int argc, char **argv)
{
    // 将第一个参数作为视频路径，读取视频
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <video_path>" << endl;
        return -1;
    }

    VideoCapture cap(argv[1]); // 打开指定视频文件
    //! 设置视频的帧数
    cap.set(CAP_PROP_FPS, 60);

    if (!cap.isOpened())
    {
        cerr << "Error: Could not open video file." << endl;
        return -1;
    }
    Mat frame;

    // 获取视频总帧数
    int total_frames = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    int current_frame = 0;

    // 创建进度条窗口
    const string trackbar_window = "Debug";
    WindowAutoLayout::get()->addWindow(trackbar_window);
    resizeWindow(trackbar_window, 640, 480);

    // 回调函数用于拖动进度条时跳转到指定帧
    auto on_trackbar = [](int pos, void *userdata)
    {
        VideoCapture *cap_ptr = static_cast<VideoCapture *>(userdata);
        cap_ptr->set(CAP_PROP_POS_FRAMES, pos);
    };
    createTrackbar("Frame", trackbar_window, &current_frame, max(1, total_frames - 1), on_trackbar, &cap);
    vector<FeatureNode_ptr> rune_groups{};

    while (true)
    {
        cap.read(frame); // 从摄像头捕获一帧
        if (frame.empty())
        {
            cerr << "Error: Could not read frame." << endl;
            // 从头开始播放
            cap.set(CAP_PROP_POS_FRAMES, 0);
            current_frame = 0;
            continue;
        }

        DebugTools::get()->setImage(frame);

        DetectorInput input;
        DetectorOutput output;
        input.setImage(frame);
        input.setTick(cv::getTickCount());
        input.setGyroData(GyroData()); // 空数据
        input.setColor(PixChannel::RED);
        input.setFeatureNodes(rune_groups);

        static auto rune_detector = RuneDetector::make_detector();

        rune_detector->detect(input, output);
        rune_groups = output.getFeatureNodes();

        // 获取待击打靶心
        do
        {
            if (rune_groups.empty())
                break;
            auto rune_group = RuneGroup::cast(rune_groups.front());
            if (rune_group->childFeatures().empty())
                break;
            FeatureNode_cptr target_tracker = nullptr;
            for (auto tracker : rune_group->getTrackers())
            {
                auto tracker_ = TrackingFeatureNode::cast(tracker);
                if (tracker_->getHistoryNodes().size() < 2)
                    continue;
                auto type = RuneCombo::cast(tracker_->getHistoryNodes().front())->getRuneType();
                if (type == RuneType::PENDING_STRUCK)
                {
                    target_tracker = tracker;
                    break;
                }
            }
            if (!target_tracker)
                break;
            // 获取位姿信息
            auto pose = target_tracker->getPoseCache().getPoseNodes().at(CoordFrame::CAMERA);
            auto tvec = pose.tvec();
            auto x = tvec[0];
            auto y = tvec[1];
            auto z = tvec[2];
            auto dist = sqrt(x * x + y * y + z * z);
            DebugTools::get()->getPVM()->addParam("Target", "Dist", dist);
        } while (0);

        auto img_show = DebugTools::get()->getImage();

        imshow(trackbar_window, img_show);

        DebugTools::get()->show();
        int key = waitKey(5);
        if (key == 27) // 按 'ESC' 键退出
        {
            break;
        }
        else if (key == 32) // 空格暂停
        {
            while (true)
            {
                int key2 = waitKey(0);
                if (key2 == 32) // 再次空格继续
                    break;
                else if (key2 == 27)
                    return 0;
            }
        }
    }
}
