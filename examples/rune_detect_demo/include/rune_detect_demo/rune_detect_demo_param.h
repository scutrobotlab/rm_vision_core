#pragma once

#include<string>

struct RuneDetectDemoParam
{
    //! 识别颜色的二值化阈值
    int color_thresh = 50;
    //! 识别颜色的类型(红色:0,蓝色:1)
    int color_type = 0;
    //! 视频路径
    std::string video_path = "";
    //! 是否可获取视频总帧数
    bool is_get_total_frames = false;
    //! 视频总帧数
    int total_frames = 0;
    //! 是否可获取视频帧率
    bool is_get_fps = false;
    //! 视频帧率
    double fps = 0.0;
};
extern RuneDetectDemoParam rune_detect_demo_param;