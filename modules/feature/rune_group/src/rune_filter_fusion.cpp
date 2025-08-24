#include "vc/feature/rune_filter_fusion.h"
#include "vc/feature/rune_filter_ekf.h"

using namespace std;
using namespace cv;

std::shared_ptr<RuneFilterStrategy> RuneFilterFusion::make_filter()
{
    return make_shared<RuneFilterFusion>();
}

auto RuneFilterFusion::filter(FilterInput input) -> FilterOutput
{
    auto &raw = input.raw_pos;
    auto &tick = input.tick;
    auto &cam = input.cam_to_gyro;
    auto &obs = input.is_observation;

    Matx61d result{};

    if (!_is_init_filter)
    {
        initFilter(raw, tick);
        _is_init_filter = true;
        result = raw;
    }
    else
    {
        unordered_map<RuneFilter_ptr, Matx61d> res_map;
        for (auto &filt : _filters)
        {
            FilterInput fi{raw, tick, cam, obs};
            res_map[filt] = filt->filter(fi).filtered_pos;
        }

        unordered_map<RuneFilterDataType, vector<RuneFilter_ptr>> type_map;
        for (auto &filt : _filters)
            type_map[filt->getDataType()].push_back(filt);

        auto &xyz = type_map[RuneFilterDataType::XYZ];
        auto &yp = type_map[RuneFilterDataType::YAW_PITCH];
        auto &roll = type_map[RuneFilterDataType::ROLL];

        result(0) = xyz.empty() ? raw(0) : res_map[xyz.front()](0);
        result(1) = xyz.empty() ? raw(1) : res_map[xyz.front()](1);
        result(2) = xyz.empty() ? raw(2) : res_map[xyz.front()](2);

        result(3) = yp.empty() ? raw(3) : res_map[yp.front()](3);
        result(4) = yp.empty() ? raw(4) : res_map[yp.front()](4);

        result(5) = roll.empty() ? raw(5) : res_map[roll.front()](5);
    }

    __filter_count++;
    __latest_value = result;
    return FilterOutput{result};
}

Matx61d RuneFilterFusion::getPredict()
{
    RuneFilter_ptr x = nullptr, y = nullptr, r = nullptr;
    for (auto &f : _filters)
    {
        switch (f->getDataType())
        {
        case RuneFilterDataType::XYZ:
            x = f;
            break;
        case RuneFilterDataType::YAW_PITCH:
            y = f;
            break;
        case RuneFilterDataType::ROLL:
            r = f;
            break;
        default:
            break;
        }
    }

    if (!x || !y || !r)
        VC_THROW_ERROR("滤波器未初始化");
    if (!x->isValid() || !y->isValid() || !r->isValid())
        VC_THROW_ERROR("滤波器无效");

    auto px = x->getPredict();
    auto py = y->getPredict();
    auto pr = r->getPredict();
    return Matx61d(px(0), px(1), px(2), py(3), py(4), pr(5));
}

void RuneFilterFusion::initFilter(const Matx61d &, int64)
{
    _filters.push_back(RuneFilterEKF_CV::make_filter(RuneFilterDataType::XYZ));
    _filters.push_back(RuneFilterEKF_CV::make_filter(RuneFilterDataType::YAW_PITCH));
    _filters.push_back(RuneFilterEKF_CV::make_filter(RuneFilterDataType::ROLL));
}

bool RuneFilterFusion::isValid()
{
    if (_filters.size() == 1)
        return _filters.front()->isValid();

    RuneFilter_ptr x = nullptr, y = nullptr, r = nullptr;
    for (auto &f : _filters)
    {
        switch (f->getDataType())
        {
        case RuneFilterDataType::XYZ:
            x = f;
            break;
        case RuneFilterDataType::YAW_PITCH:
            y = f;
            break;
        case RuneFilterDataType::ROLL:
            r = f;
            break;
        default:
            break;
        }
    }
    return x && y && r && x->isValid() && y->isValid() && r->isValid();
}
