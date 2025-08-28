#include "include/rune_detect_demo/rune_detect_demo.h"
#include "include/rune_detect_demo/rune_detect_demo_param.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    parseCommandLine(argc, argv);
    VC_PASS_INFO("Video Path: %s", rune_detect_demo_param.video_path.c_str());
    VideoCapture cap(rune_detect_demo_param.video_path);
    if (!cap.isOpened())
        VC_THROW_ERROR("无法打开视频文件: %s", rune_detect_demo_param.video_path.c_str());
    setupVideoCapture(cap);
    while (true)
    {
        process(cap);
        DebugTools::get()->show();
        waitKey(1);
    }
}
