#include "vc/core/debug_tools/param_view_manager.h"
#include "vc/core/yml_manager.hpp"

struct ParamViewManagerParam
{
    //! 画布的行数
    int img_row = 600;
    //! 画布的列数
    int img_col = 800;
    //! 最大点数
    int max_points = 300;

    YML_INIT(ParamViewManagerParam,
             YML_ADD_PARAM(img_row);
             YML_ADD_PARAM(img_col);
             YML_ADD_PARAM(max_points););
};
inline ParamViewManagerParam param_view_manager_param;

ParamViewManager::ParamViewManager() {}

void ParamViewManager::addParam(const std::string &canvas_name, const std::string &param_key, float value)
{
    auto it = canvas_map_.find(canvas_name);
    if (it == canvas_map_.end())
    {
        auto canvas = std::make_shared<ParamCanvas>(canvas_name,
                                                    param_view_manager_param.img_col,
                                                    param_view_manager_param.img_row,
                                                    param_view_manager_param.max_points);
        canvas_map_[canvas_name] = canvas;
        it = canvas_map_.find(canvas_name);
    }

    it->second->addParam(param_key, value);
}

void ParamViewManager::addParam(const std::string &canvas_name, const std::string &param_key, const std::vector<float> &values, bool overwrite)
{
    auto it = canvas_map_.find(canvas_name);
    if (it == canvas_map_.end())
    {
        auto canvas = std::make_shared<ParamCanvas>(canvas_name,
                                                    param_view_manager_param.img_col,
                                                    param_view_manager_param.img_row,
                                                    param_view_manager_param.max_points);
        canvas_map_[canvas_name] = canvas;
        it = canvas_map_.find(canvas_name);
    }

    it->second->addParam(param_key, values, overwrite);
}

void ParamViewManager::resetCanvas(const std::string &canvas_name)
{
    auto it = canvas_map_.find(canvas_name);
    if (it != canvas_map_.end())
        it->second->reset();
}

void ParamViewManager::resetAll()
{
    for (auto &[_, canvas] : canvas_map_)
    {
        canvas->reset();
    }
}

void ParamViewManager::showAll()
{
    for (auto &[_, canvas] : canvas_map_)
    {
        canvas->show();
    }
}

void ParamViewManager::setCanvasSize(const std::string &canvas_name, int width, int height)
{
    auto it = canvas_map_.find(canvas_name);
    if (it != canvas_map_.end())
        it->second->setCanvasSize(width, height);
}

void ParamViewManager::setCanvasTicks(const std::string &canvas_name, int x_ticks, int y_ticks)
{
    auto it = canvas_map_.find(canvas_name);
    if (it != canvas_map_.end())
        it->second->setTicks(x_ticks, y_ticks);
}

void ParamViewManager::setCanvasMaxPoints(const std::string &canvas_name, int max_points)
{
    auto it = canvas_map_.find(canvas_name);
    if (it != canvas_map_.end())
        it->second->setMaxPoints(max_points);
}
