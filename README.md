# SCAU Taurus 2025 Fall 视觉组考核完成情况总结

- 填写者：张坤
- 数学与信息学院 / 2023 级计算机科学与技术 3 班 / 202325310328

## 开发环境

- 系统：Ubuntu 22.04 on WSL2 (Kernel: 6.6.87.2-1)
- 运行时：
  - OpenCV: 4.11.0
  - opencv-contrib: 4.11.0
  - ROS2: humble
- 工具链：
  - CMake 3.22.1
  - Make 4.3
  - colcon (core: 0.20.0)
  - g++ 11.4.0
  - gdb 12.1
  - Optional:
    - clangd 12.0.1
    - clang-format 14.0.0

## 项目结构

``` 
.
├── README.md
├── 赛事知识题.md
├── asset
└── src
    ├── cv_related
    │   ├── CMakeLists.txt
    │   ├── apple_detect.cpp  # 苹果轮廓提取
    │   ├── armor.cpp         # 装甲板对象设计
    │   ├── calibration.cpp   # 相机标定
    │   ├── color_matcher.cpp # 色块提取
    │   ├── defog.cpp         # 图片去雾
    │   ├── mouse_event.cpp   # 鼠标事件
    │   ├── recorder.cpp      # 视频录制
    │   ├── res               # cv_related 中代码运行时用得到的一些资源
    │   └── utils             # 杂项代码，用于调试OpenCV的配置
    └── ros_related
        ├── res                    # armor_pipeline 功能包中节点运行时用得到的一些资源
        │   └── pipe_params.yaml   # armor_pipeline 功能包中节点的参数
        └── src
            ├── armor_detect   # ROS无关的装甲检测算法库，用colcon打包
            ├── armor_pipeline # 装甲检测节点
            ├── cv_topic       # 原数据类型为cv::Mat的图像话题
            └── str_topic      # 原数据类型为std::string的字符串话题

```
## 考核完成度
1. 赛事知识题：全部完成
2. C++应用题：全部完成
3. ROS2基本应用：全部完成
4. OpenCV:
    - 基础题：全部完成
    - 应用题：全部完成
    - 赛事题：
        - 装甲板识别：仅未完成测距
        - 能量机关识别：未完成
5. 深度学习：未完成
## 考核完成思路
