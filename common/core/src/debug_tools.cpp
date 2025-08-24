#include "vc/core/debug_tools.h"
#include "vc/core/debug_tools/param_view_manager.h"
#include "vc/core/debug_tools/window_auto_layout.h"

using namespace std;
using namespace cv;

// 单例模式，方便全局调用
DebugTools_ptr DebugTools::get()
{
    static DebugTools_ptr instance = std::make_shared<DebugTools>();
    return instance;
}

// 设置缓存图像（通常在处理开始时调用）
void DebugTools::setImage(const cv::Mat &img)
{
    std::lock_guard<std::mutex> lock(mtx_);
    img.copyTo(cache_);
}

// 获取缓存图像引用（用于绘制调试信息）
cv::Mat DebugTools::getImage()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (cache_.empty())
        return cv::Mat();
    return cache_;
}

// 在固定窗口显示
void DebugTools::show(const std::string &winName)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (!window_initialized_)
    {
        initWindow(winName);
        window_initialized_ = true;
    }

    if (!cache_.empty())
    {
        cv::imshow(winName, cache_);
    }

    if(pvm_)
        pvm_->showAll();
}


// 初始化窗口
void DebugTools::initWindow(const std::string &winName)
{
    WindowAutoLayout::get()->addWindow(winName);
    // cv::namedWindow(winName, cv::WINDOW_NORMAL);
    cv::resizeWindow(winName, 640, 480);
}

std::shared_ptr<ParamViewManager> DebugTools::getPVM()
{
    if(!pvm_)
        pvm_ = ParamViewManager::create();
    return pvm_;
}