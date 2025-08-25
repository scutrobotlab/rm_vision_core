# **【RM2025-能量机关自瞄算法开源】【神符多角点识别方案】华南理工大学** **华南虎**战队



Github 仓库地址：[rm_vision_core](https://github.com/scutrobotlab/rm_vision_core)

## 1. 核心创新点与优势

- 【新思路】我们提出了一种通用的纯色图形角点提取思路，可用于稳定提取不规则轮廓的角点。
- 【算法】基于上述思路，我们设计了一种针对局部条状轮廓的末端点提取算法。可用于稳定识别神符扇叶（已激活形态）或神符靶心的角点。角点识别效果示意可见图1.1。

<p align="center"> <img src="https://github.com/user-attachments/assets/d84b0256-8268-4271-a9c6-1aac5015d852" width="735" /> </p> <p align="center">图1.1</p>

- 【优势】相对于多边形近似和普通凸包检测的传统角点识别思路，该算法提升了稳定性和精度，角点提取误差稳定在`1~3`个像素点以内。一次开符过程最多可同时识别`37`个稳定角点。

## 2. 功能介绍

- 本项目是一套面向 **能量机关（神符）多角点识别与自瞄** 的识别算法框架，涵盖神符识别中特征识别、角点提取、位姿解算的全链路功能。整体采用模块化设计，便于移植、优化与二次开发。
- 核心实现基于 **C++**，并使用 **OpenCV 4.7.0** 作为图像处理引擎，无需依赖 ROS，环境配置简单高效。
- 该项目构建于 **华南虎 视觉库 SRVL** 的基础之上，是面向开源需求的 特化版本。在保持识别逻辑一致的前提下，移除了大量冗余代码，优化了架构与注释，提高了可读性与可维护性，更适合开源社区交流与二次开发。
- 源码实现包含本队在神符自瞄中的完整识别逻辑，涵盖（但不限于）：神符靶心角点提取算法、神符扇叶角点提取算法、多特征匹配算法、位姿（PnP）解算模块。此外，项目还提供多种辅助开发工具，如：参数配置自动生成器，快速生成标准化配置文件；参数可视化管理器，支持多维度调试与曲线对比；日志信息打印系统，方便调试与性能分析。
- 算法原理与具体实现细节会在后文详细介绍。

## 3. 效果展示

- 展示内容可分为三部分：

1. 整体角点识别效果展示。
2. 扇叶特征的抗遮挡识别效果展示。
3. 实车测试数据展示。

- 识别效果的优劣将通过 **重投影验证** 的方式展示：成功识别的角点会被重新绘制到图像上，便于通过肉眼直观评估识别精度。 此外，识别过程的稳定性也会通过 **曲线可视化** 的形式进行展示，直观呈现角点位置或姿态解算在连续帧中的平滑性与一致性。
- 需要说明：由于资源条件限制，除**实车测试**部分外，其余测试过程均在**无车身 IMU 姿态信息**的情况下进行，这意味着部分位姿解算逻辑未被启用。因此，当前展示的离线识别效果相比实车部署会略有折扣；在实车环境下启用完整的姿态补偿后，整体识别与解算性能会更佳。

### 3.1 整体角点识别效果

  识别成功的角点会通过浅绿色圆圈标记角点所在位置，并且会在图像上用白色字体标出序号。相邻的角点还会使用细线进行连接。

### 3.2 扇叶特征的抗遮挡识别

  该效果视频用于展示 **扇叶特征在部分遮挡情况下的识别能力**。 当机器人从侧方观测神符时，受中央凸台结构的影响，远端扇叶往往会被部分遮挡，导致暴露在视野中的图案仅剩 2~3 块不规则灯条。在这种情况下，算法会**自适应降低识别角点数量**，以保证整体识别的稳定性与连续性，从而确保后续位姿解算和决策的可靠性。

  如 **视频 3.2** 所示：

- **扇叶完全可见时**，识别出的角点数为 **6 个**，连线近似构成一个规则的五边形；
- **扇叶部分可见时**，识别角点数会动态调整为 **3 个**，连线近似构成一个三角形。

  此外，当扇叶轮廓因 **光晕、拖影等因素** 变得模糊时，该抗遮挡策略同样会被触发。这本质上是一种面向极端识别环境的 **稳健角点提取机制**，在识别条件受限时尽可能输出稳定、可用的角点数据。

### 3.3 实车测试数据展示

  通过分析视频3.3中的识别曲线，可以清楚地看到：

- **单靶心亮起（无扇叶识别状态）** 时，缺乏足够的角点约束条件，导致位姿解算出现明显抖动；
- 当 **第二块靶心亮起（有扇叶识别状态）** 后，识别算法能够提取到扇叶特征点，解算结果迅速稳定；
- 随着靶心数量进一步增加，识别角点更为丰富，解算的精度和稳定性也随之显著提升。

  通过引入 **扇叶角点提取** 作为额外特征约束，算法在部分靶心不可见或信号不足的情况下，依旧能够维持较高的位姿解算精度和稳定性。这一改进有效提升了后续控制与决策模块的可靠性。

## 4. 项目信息

### 4.1 系统框图

系统框图如图4.1.1  
<p align="center"> <img width="800" alt="Image" src="https://github.com/user-attachments/assets/878ef41f-5cb8-4fc4-a9b9-5296513ed0f0" /> </p> <p align="center">图4.1.1</p>


### 4.2 软件架构

类型封装和属性定义采用自定义宏库规范生成和管理。且针对大部分常用信息结构都进行了封装（位姿节点、轮廓属性等）。  

<p align="center"> <img width="800" alt="Image" src="https://github.com/user-attachments/assets/aa17d638-eedf-49d7-aef5-8aba4f51fce3" /> </p> <p align="center">图4.2.1</p>

### 4.3 数据流图

 整体设计采用逐层组装的思路：从基础特征的识别入手，逐步向上构建更高层级的结构，并在每一层解算所需的计算信息。各特征节点之间统一通过 `FeatureNode_ptr` 接口进行连接和交互。
 
<p align="center"> <img width="800" alt="Image" src="https://github.com/user-attachments/assets/e20a4e1d-419f-492c-8ee1-fbd6067fcbb4" /> </p> <p align="center">图4.3.1</p>


### 4.4 文件结构

```Plain
.
├── CMakeLists.txt                # 顶层CMake构建脚本，定义整个工程的编译入口
├── bin                           # 存放生成的可执行文件
├── cmake
│   ├── VisCoreCompilerOptions.cmake   # 全局编译参数配置，如C++标准、优化选项
│   └── VisCoreModule.cmake            # 子模块构建和依赖加载脚本
├── common                         # 公共库，封装底层功能与通用工具
│   ├── CMakeLists.txt              # 公共库构建脚本
│   ├── camera                      # 相机参数与接口
│   │   ├── CMakeLists.txt
│   │   └── include
│   │       └── vc
│   │           └── camera
│   │               ├── camera_param.h       # 相机参数结构定义与接口
│   │               └── yml
│   │                   └── CameraParam.yml  # 默认相机内参、畸变参数配置
│   ├── contour_proc                 # 轮廓处理工具
│   │   ├── CMakeLists.txt
│   │   └── include
│   │       └── vc
│   │           └── contour_proc
│   │               ├── contour_converter.hpp # 轮廓格式转换工具
│   │               ├── contour_wrapper.hpp   # 轮廓封装接口
│   │               ├── extensions.hpp        # 轮廓相关扩展功能
│   │               └── hierarchy.hpp         # 轮廓层级管理工具
│   ├── core                        # 核心功能和调试工具
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── vc
│   │   │       └── core
│   │   │           ├── cv_expansion_type.h   # OpenCV类型扩展定义
│   │   │           ├── debug_tools           # 调试与可视化工具接口
│   │   │           │   ├── param_canvas.h       # 参数曲线绘制工具
│   │   │           │   ├── param_view.h         # 参数单视图显示接口
│   │   │           │   ├── param_view_manager.h # 多视图管理接口
│   │   │           │   └── window_auto_layout.h # 窗口自动布局接口
│   │   │           ├── debug_tools.h        # 调试工具入口头文件
│   │   │           ├── logging.h            # 日志接口
│   │   │           ├── property_wrapper.hpp # 属性包装工具
│   │   │           ├── type_expansion.hpp   # 类型扩展工具
│   │   │           ├── type_utils.h         # 类型辅助函数
│   │   │           └── yml_manager.hpp      # YML参数解析与管理
│   │   └── src
│   │       ├── core.cpp                     # 核心功能实现
│   │       ├── debug_tools
│   │       │   ├── param_canvas.cpp         # 参数曲线绘制实现
│   │       │   ├── param_view_manager.cpp   # 多视图管理实现
│   │       │   ├── window_auto_layout.cpp   # 窗口自动布局实现
│   │       │   └── yml
│   │       │       └── ParamViewManagerParam.yml # 参数视图管理配置文件
│   │       └── debug_tools.cpp              # 调试工具入口实现
│   ├── dataio                     # 数据输入输出模块
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── vc
│   │   │       └── dataio
│   │   │           └── dataio.h             # 数据IO接口头文件
│   │   └── src
│   │       └── dataio.cpp                   # 数据IO实现
│   ├── math                        # 数学工具模块
│   │   ├── CMakeLists.txt
│   │   └── include
│   │       └── vc
│   │           └── math
│   │               ├── geom
│   │               │   ├── geom_circle.hpp   # 圆几何定义
│   │               │   ├── geom_line.hpp     # 直线几何定义
│   │               │   └── geom_segment.hpp  # 线段几何定义
│   │               ├── geom_utils.hpp        # 几何通用工具函数
│   │               ├── geometry.h            # 通用几何接口
│   │               ├── pose_node.hpp         # 位姿节点数据结构
│   │               ├── transform6D.hpp       # 六自由度变换工具
│   │               └── type_utils.hpp        # 数学类型辅助工具
│   └── modules                     # 公共模块
│       ├── CMakeLists.txt
│       ├── detector
│       │   ├── CMakeLists.txt
│       │   └── include
│       │       └── vc
│       │           └── detector
│       │               ├── detector.h        # 通用识别器接口
│       │               ├── detector_input.h  # 识别器输入数据定义
│       │               └── detector_output.h # 识别器输出数据定义
│       └── feature
│           ├── CMakeLists.txt
│           └── include
│               └── vc
│                   └── feature
│                       ├── feature_node.h                  # 特征节点通用定义
│                       ├── feature_node_child_feature_type.h # 子特征类型定义
│                       ├── feature_node_draw_config.h      # 特征节点绘制参数
│                       └── tracking_feature_node.h         # 可追踪特征节点定义
├── examples                      # 示例工程
│   ├── CMakeLists.txt
│   └── rune_detect_demo
│       ├── CMakeLists.txt
│       ├── include
│       │   └── rune_detect_demo
│       │       └── rune_detect_demo.h        # 示例程序头文件
│       ├── main.cpp                          # 示例入口
│       └── src
│           └── rune_detect_demo.cpp           # 示例逻辑实现
└── modules                       # 高层功能模块
    ├── CMakeLists.txt
    ├── detector
    │   ├── CMakeLists.txt
    │   └── rune_detector
    │       ├── CMakeLists.txt
    │       ├── include
    │       │   └── vc
    │       │       └── detector
    │       │           ├── rune_detector.h         # 扇叶识别器接口
    │       │           ├── rune_detector_param.h   # 扇叶识别器参数定义
    │       │           └── yml
    │       │               └── RuneDetectorParam.yml # 扇叶识别器配置参数
    │       └── src
    │           ├── rune_detector.cpp             # 扇叶识别器核心实现
    │           ├── rune_detector_calc.cpp        # 识别计算模块
    │           ├── rune_detector_find.cpp        # 候选区域搜索模块
    │           ├── rune_detector_get_pnp_data.cpp# PnP求解数据接口
    │           ├── rune_detector_get_runes.cpp   # 扇叶特征提取
    │           ├── rune_detector_match.cpp       # 匹配逻辑实现
    │           └── yml
    │               └── RuneDetectorFilterCenterParam.yml # 过滤中心参数
    └── feature
        ├── CMakeLists.txt
        ├── rune_center
        │   ├── CMakeLists.txt
        │   ├── include
        │   │   └── vc
        │   │       └── feature
        │   │           ├── rune_center.h           # R标识别接口
        │   │           ├── rune_center_param.h     # R标参数定义
        │   │           └── yml
        │   │               ├── RuneCenterDrawParam.yml # 绘制参数
        │   │               └── RuneCenterParam.yml     # 参数配置
        │   └── src
        │       └── rune_center.cpp                # R标识别实现
        ├── rune_combo
        │   ├── CMakeLists.txt
        │   ├── include
        │   │   └── vc
        │   │       └── feature
        │   │           ├── rune_combo.h           # 扇叶组合接口
        │   │           └── rune_combo_param.h     # 扇叶组合参数
        │   └── src
        │       └── rune_combo.cpp                 # 扇叶组合实现
        ├── rune_fan
        │   ├── CMakeLists.txt
        │   ├── include
        │   │   └── vc
        │   │       └── feature
        │   │           ├── rune_fan.h             # 扇叶基础接口
        │   │           ├── rune_fan_active.h      # 已激活扇叶接口
        │   │           ├── rune_fan_hump.h        # 扇叶凸起识别接口
        │   │           ├── rune_fan_hump_param.h  # 凸起识别参数
        │   │           ├── rune_fan_inactive.h    # 非已激活扇叶接口
        │   │           ├── rune_fan_param.h       # 扇叶参数定义
        │   │           └── yml
        │   │               ├── RuneFanDrawParam::Active.yml   # 已激活扇叶绘制参数
        │   │               ├── RuneFanDrawParam::Inactive.yml # 非已激活扇叶绘制参数
        │   │               ├── RuneFanHumpParam.yml           # 凸起识别参数
        │   │               └── RuneFanParam.yml               # 通用参数
        │   └── src
        │       ├── hump_bottom_center.cpp         # 下部凸起识别实现
        │       ├── hump_detector.cpp              # 凸起识别主逻辑
        │       ├── hump_side.cpp                  # 侧面凸起识别实现
        │       ├── hump_top.cpp                   # 顶部凸起识别实现
        │       ├── rune_fan_active.cpp            # 已激活扇叶实现
        │       ├── rune_fan_active_incomplete.cpp # 残缺已激活扇叶实现
        │       ├── rune_fan_common.cpp            # 扇叶通用逻辑
        │       ├── rune_fan_inactive.cpp          # 非已激活扇叶实现
        │       └── yml
        │           └── TopHumpDebugParam.yml      # 顶部凸起调试参数
        ├── rune_group
        │   ├── CMakeLists.txt
        │   ├── include
        │   │   └── vc
        │   │       └── feature
        │   │           ├── rune_data_converter.h  # 扇叶数据转换接口
        │   │           ├── rune_filter_ekf.h      # EKF滤波接口
        │   │           ├── rune_filter_ekf_param.h# EKF滤波参数
        │   │           ├── rune_filter_fusion.h   # 多滤波融合接口
        │   │           ├── rune_filter_type.h     # 滤波类型定义
        │   │           ├── rune_group.h           # 扇叶组管理接口
        │   │           ├── rune_group_filter.h    # 扇叶组滤波接口
        │   │           ├── rune_group_param.h     # 扇叶组参数
        │   │           └── yml
        │   │               ├── RuneEKFParam.yml   # EKF滤波配置
        │   │               └── RuneGroupParam.yml # 扇叶组参数配置
        │   └── src
        │       ├── data_converter.cpp             # 数据转换实现
        │       ├── pnp_calc.cpp                   # PnP位姿计算实现
        │       ├── rune_filter_ekf.cpp            # EKF滤波实现
        │       ├── rune_filter_fusion.cpp         # 滤波融合实现
        │       └── rune_group.cpp                 # 扇叶组管理实现
        ├── rune_target
        │   ├── CMakeLists.txt
        │   ├── include
        │   │   └── vc
        │   │       └── feature
        │   │           ├── rune_target.h          # 扇叶靶心接口
        │   │           ├── rune_target_active.h   # 已激活靶心接口
        │   │           ├── rune_target_inactive.h # 非已激活靶心接口
        │   │           ├── rune_target_param.h    # 扇叶靶心参数
        │   │           └── yml
        │   │               ├── RuneTargetDrawParam::Active.yml   # 已激活靶心绘制参数
        │   │               ├── RuneTargetDrawParam::Inactive.yml # 非已激活靶心绘制参数
        │   │               └── RuneTargetParam.yml               # 扇叶靶心配置
        │   └── src
        │       ├── rune_target_active.cpp         # 已激活靶心实现
        │       ├── rune_target_common.cpp         # 靶心通用逻辑
        │       └── rune_target_inactive.cpp       # 非已激活靶心实现
        └── rune_tracker
            ├── CMakeLists.txt
            ├── include
            │   └── vc
            │       └── feature
            │           ├── rune_tracker.h         # 神符追踪单元接口
            │           ├── rune_tracker_param.h   # 神符追踪单元参数
            │           └── yml
            │               └── RuneTrackerParam.yml # 神符追踪单元配置
            └── src
                └── rune_tracker.cpp               # 神符追踪单元实现
```

### 4.5 项目环境

- 操作系统：Ubuntu 22.04
- 计算平台：机械君锐龙Ren5000（R7-5800H-16核）
- 相机型号：海康MV-CS016
- 镜头型号：海康官方5mm镜头
- 通信接口：CH340 USB 转串口模块

### 4.6 编译方式

- C++20、OpenCV 4.70 、 Eigen3

## 5. 扇叶角点识别原理

### 5.1 条状突起检测

- 轮廓上每一个点都有自己的方向。且可以通过前一个轮廓点坐标 减 后一个轮廓点坐标 求得该方向。
- 灯条同一侧的轮廓点基本位于一条直线上。
- 灯条同一侧的轮廓点的方向基本相同。
- 灯条两侧的轮廓点的方向基本相反。
- 灯条两侧的轮廓点 在其垂线上的投影点的距离差基本相等。（反映了灯条的厚度）
- 通过以上特点，我们可以尝试为一个轮廓上的某一个点查找其反向匹配点，（即假设当前点位于灯条的一侧时，尝试为其匹配灯条另一侧的点）若成功查找到匹配点，则说明该点位于灯条上，否则不在灯条上。
- 合并一定范围内所有匹配成功的轮廓点，就可以得到一对线段。（由 上升线段 和 下降线段组成。前者由正向点合并得到、后者由下降点合并得到），两条线段求平均，就可以得到灯条的中轴线。
  
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/1fb40ce2-2d58-4aff-a53a-4e5d64da4390" width="150"></td>
    <td><img src="https://github.com/user-attachments/assets/ad9b73ed-db26-48fc-b8f4-ec1d8e027cbb" width="150"></td>
    <td><img src="https://github.com/user-attachments/assets/e5fec197-0ea2-42a7-8d10-80a765c6edfd" width="150"></td>
    <td><img src="https://github.com/user-attachments/assets/57bd5fe0-8e49-4ca8-883d-b4a426751e17" width="150"></td>
    <td><img src="https://github.com/user-attachments/assets/1cb3fd9e-aa58-4fff-805d-25aedd809b98" width="150"></td>
  </tr>
  <tr>
    <td align="center">图5.1.1</td>
    <td align="center">图5.1.2</td>
    <td align="center">图5.1.3</td>
    <td align="center">图5.1.4</td>
    <td align="center">图5.1.5</td>
  </tr>
</table>


### 5.2 轮廓的链码化

- 链码是一种用方向序列来表示轮廓的方法。假设我们有一个由像素点组成的物体边界，链码会按照一定顺序（如顺时针）遍历边界像素，并用 **数字编码当前像素指向下一个像素的方向**。参考图5.2.1 [1] 的效果。
 
<p align="center">
  <img src="https://github.com/user-attachments/assets/89deba52-736d-43ab-93b1-d026243744bd" width="572" />
</p>
<p align="center">图5.2.1</p>

- 我们先对扇叶进行二值化处理，得到图5.2.2
 
<p align="center">
  <img src="https://github.com/user-attachments/assets/841da63e-f3cd-4f90-8570-17cca47db06f" width="233" />
</p>
<p align="center">图5.2.2</p>

- 对其提取轮廓，并将轮廓点链码化，将链码数组可视化成曲线，即可得到图5.2.3。由于我们使用的是4连通链码 [2]，因此可以明显看到链码表被清晰的分成了四个层级。一个层级代表一个方向。

<p align="center">
  <img src="https://github.com/user-attachments/assets/05b91555-01a5-4bfe-9c8c-89b6b60ab57a" width="800" />
</p>
<p align="center">图5.2.3</p>

- 我们将链码值映射为角度（0~360°），并进行滤波操作，即可得到扇叶轮廓的方向角的角度值数组。如图5.2.4。其中，我们采用`cv::Mat` 作为角度存储的容器，方便使用 OpenCV 提供的自定义滤波器求解梯度矩阵。

<p align="center">
  <img src="https://github.com/user-attachments/assets/03c973c7-406c-4b63-9a5d-095a114ed269" width="800" />
</p>
<p align="center">图5.2.4</p>

- 通过该角度数组，我们能够初步提取一些关键信息。例如，在图5.2.5中，红、绿两条线所在的角度平面相差180°，且它们之间的角度呈单调变化。这表明它们对应的轮廓点极有可能属于同一个凸起块。

<p align="center">
  <img src="https://github.com/user-attachments/assets/5342356d-0b92-4e3e-8e0b-a87a8dd3dcf1" width="800" />
</p>
<p align="center">图5.2.5</p>

### 5.3 邻点聚类压缩

- 在前文提到的条状突起检测算法中，提取正反线段时受限于轮廓点数量，导致重复计算频繁、性能低下。优化策略是将方向一致、分布近似直线的角度点合并，并基于点数赋权，实现轮廓点的批量压缩。这一过程相当于先通过拟合直线去除冗余信息，从而提升后续反向点匹配与查找的效率。
- 上升线段的识别效果如视频5.3.1

- 获取到正反线段对后，仅需要套用装甲板灯条的识别思路即可完成角点的提取。具体逻辑实现可参考源码文件。

## 6. 未来优化方向

- 本赛季的二值化流程仍较为原始，采用逐帧全图二值化，并依赖手动调整曝光阈值，受光照变化影响较大。虽然已有优化方案，但由于时间限制未能落地实现。
- 计划在后续版本中引入神经网络模型。目前基于深度学习的姿态推理技术已相当成熟，有望提升算法的鲁棒性与适应性。

## 7. 项目成员

- 开发人员：张峰玮
- 指导老师：黄沿江、杨丽新、李海。 
- 技术指导：黄星蔚、刘玮涵、邱振华，以及虎虎的历代视觉老队员们
- 特别鸣谢：
  - 三天四趟跑腿，为了修符，拧螺丝拧到麻木的曾博同学
  - 大符出厂那几天，板符板焊到手抽筋的硬件组队友们
  - 提供部分算法思路的华工智能车队刘展华同学

## 8. 参考文献

[1] [数字图像处理学习笔记——表示与描述](https://zhuanlan.zhihu.com/p/634675543)

[2] [CSDN 表示与描述——链码讲解](https://blog.csdn.net/qq_63029071/article/details/140361695)

## 9. 通讯信息

微信：zfw_yefan
