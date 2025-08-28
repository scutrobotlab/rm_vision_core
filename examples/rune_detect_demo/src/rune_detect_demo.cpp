#include "../include/rune_detect_demo/rune_detect_demo.h"
#include "../include/rune_detect_demo/rune_detect_demo_param.h"
#include "vc/core/debug_tools/window_auto_layout.h"

using namespace std;
using namespace cv;

void process(cv::VideoCapture &vid_cap)
{
    static auto rune_groups = vector<FeatureNode_ptr>{};
    static auto rune_detector = RuneDetector::make_detector();

    // 读取识别参数
    updateParam(vid_cap);

    Mat frame;
    vid_cap.read(frame); // 从摄像头捕获一帧
    if (frame.empty())
    {
        return;
    }
    DebugTools::get()->setImage(frame);
    DetectorInput input;
    DetectorOutput output;
    input.setImage(frame);
    input.setGyroData(GyroData()); // 空数据
    input.setTick(cv::getTickCount());
    input.setColor(rune_detect_demo_param.color_type == 0 ? PixChannel::RED : PixChannel::BLUE);
    input.setColorThresh(rune_detect_demo_param.color_thresh);
    input.setFeatureNodes(rune_groups);
    rune_detector->detect(input, output);
    rune_groups = output.getFeatureNodes();
    if (rune_groups.empty())
        return;
    auto rune_group = RuneGroup::cast(rune_groups.front());
    if (rune_group->childFeatures().empty())
        return;
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
        return;
    // 绘制
    Mat img_show = DebugTools::get()->getImage();
    rune_group->drawFeature(img_show);
}

void updateParam(cv::VideoCapture &cap)
{
    // 计数器
    static int count = 0;
    static std::string winname = "param";
    static int color_type_static = 0;
    static int color_thresh_static = 160;
    count++;

    // 判断窗口是否初始化
    auto layout = WindowAutoLayout::get();
    layout->addWindow(winname);

    // 判断参数滑动条是否存在
    static bool has_init_param = false;
    if (layout->hasWindow(winname) && !has_init_param)
    {
        //! 颜色类型参数类型
        std::string color_type_param_name = "Color Type (0: Red, 1: Blue)";
        //! 颜色二值化阈值
        std::string color_thresh_param_name = "Color Thresh (0~255)";
        createTrackbar(color_type_param_name, winname, nullptr, 1, [](int pos, void *userdata)
                       {
                         int *color_type_static_ptr = static_cast<int *>(userdata);
                         *color_type_static_ptr = pos; }, &color_type_static);
        setTrackbarPos(color_type_param_name, winname, 0);
        createTrackbar(color_thresh_param_name, winname, nullptr, 255, [](int pos, void *userdata)
                       {
                           int *color_thresh_static_ptr = static_cast<int *>(userdata);
                           *color_thresh_static_ptr = pos; }, &color_thresh_static);
        setTrackbarPos(color_thresh_param_name, winname, 100);
        has_init_param = true;
    }
    // 初始化进度条
    static bool has_init_progress = false;
    static int target_pos = 0;
    static int last_set_pos = -1; // 上次设置的位置
    do
    {
        if (rune_detect_demo_param.is_get_total_frames == false)
            break;
        if (layout->hasWindow(DebugTools::get()->getWindowName()))
        {
            if (!has_init_progress)
            {
                auto total_frames = rune_detect_demo_param.total_frames;
                createTrackbar("Frame Processed", DebugTools::get()->getWindowName(), nullptr, total_frames, [](int pos, void *userdata)
                               {
                                      int *target_pos_ptr = static_cast<int *>(userdata);
                                      *target_pos_ptr = pos; }, &target_pos);
                setTrackbarPos("Frame Processed", DebugTools::get()->getWindowName(), 0);
                has_init_progress = true;
            }
            // 获取当前的进度条
            if (target_pos != last_set_pos)
            {
                // 设置视频帧位置
                cap.set(cv::CAP_PROP_POS_FRAMES, target_pos);
                last_set_pos = target_pos;
            }
            if (count % 50 == 0) // 每30帧更新一次
            {
                // 获取当前帧位置
                int current_frame = static_cast<int>(cap.get(cv::CAP_PROP_POS_FRAMES));
                setTrackbarPos("Frame Processed", DebugTools::get()->getWindowName(), current_frame);
            }
        }
        else
        {
            has_init_progress = false;
        }

        // 判断是否到达视频末尾
        if (cap.get(cv::CAP_PROP_POS_FRAMES) >= cap.get(cv::CAP_PROP_FRAME_COUNT))
        {
            // 重新开始
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            last_set_pos = -1;
            target_pos = 0;
            setTrackbarPos("Frame Processed", DebugTools::get()->getWindowName(), 0);
        }

    } while (0);

    // 进行参数赋值
    rune_detect_demo_param.color_type = color_type_static;
    rune_detect_demo_param.color_thresh = color_thresh_static;
}

void parseCommandLine(int argc, char **argv)
{
    CommandLine cli;
    // 1. 帮助项说明 ：-h, --help
    cli.addOption("-h", "help", "显示帮助信息");
    // 2. 输入视频路径 ：-i, --input <path>
    cli.addOption("-i", "input", "输入视频路径", true);

    // 解析参数
    bool valid = cli.parse(argc, argv);
    if (!valid)
    {
        cli.printHelp(argv[0]);
        VC_WARNING_INFO("程序使用示例：\n./VisCore_rune_detect_demo_exe -i ./test_video/rune_video.mp4\n注意:\n 1.视频路径为绝对路径\n 2.视频路径不要带外括号");
        exit(1);
    }

    if (cli.isSet("-h"))
    {
        cli.printHelp(argv[0]);
        exit(0);
    }

    if (cli.isSet("-i"))
    {
        rune_detect_demo_param.video_path = cli.get("-i");
        if (rune_detect_demo_param.video_path.empty())
        {
            VC_WARNING_INFO("无效的视频路径");
        }
    }

    if (cli.getPositionalArgs().size() > 0)
    {
        VC_WARNING_INFO("检测到多余的参数，已忽略");
    }
}

void setupVideoCapture(cv::VideoCapture &cap)
{
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    if (total_frames <= 0)
        VC_WARNING_INFO("无法获取视频总帧数，可能是摄像头或无效视频文件");
    else
        VC_PASS_INFO("视频总帧数: %d", total_frames);

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0)
        VC_WARNING_INFO("无法获取视频帧率，可能是摄像头或无效视频文件");
    else
        VC_PASS_INFO("视频帧率: %.2f", fps);

    bool is_get_total_frames = total_frames > 0;
    bool is_get_fps = fps > 0;
    if (!is_get_total_frames && !is_get_fps)
    {
        VC_WARNING_INFO("无法获取视频总帧数和帧率，默认按30FPS处理");
        cap.set(cv::CAP_PROP_FPS, 30);
    }
    else if (!is_get_total_frames && is_get_fps)
    {
        VC_PASS_INFO("只能获取视频帧率，按此帧率处理");
    }
    else if (is_get_total_frames && !is_get_fps)
    {
        VC_WARNING_INFO("只能获取视频总帧数，默认按30FPS处理");
        cap.set(cv::CAP_PROP_FPS, 30);
    }
    else
    {
        double duration = total_frames / fps;
        VC_PASS_INFO("视频时长: %.2f 秒", duration);
    }

    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_POS_FRAMES, 0); // 从头开始
    rune_detect_demo_param.is_get_fps = is_get_fps;
    rune_detect_demo_param.fps = fps;
    rune_detect_demo_param.is_get_total_frames = is_get_total_frames;
    rune_detect_demo_param.total_frames = total_frames;
}