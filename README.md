# SCAU Taurus 2025 Fall 视觉组考核完成情况总结
- 填写者：张坤
- 数学与信息学院 / 2023级计算机科学与技术3班 / 202325310328
## 开发环境
- 系统：Ubuntu 22.04 on WSL2 (Kernel: 6.6.87.2-1)
- 运行时：
    - OpenCV: 4.11.0 
    - opencv-contrib: 4.11.0
    - ROS2: Humble
## 项目结构
```
.
├── README.md
├── asset
└── src
    ├── cv_related
    │   ├── CMakeLists.txt
    │   ├── apple_detect.cpp
    │   ├── armor.cpp
    │   ├── calibration.cpp
    │   ├── color_matcher.cpp
    │   ├── defog.cpp
    │   ├── mouse_event.cpp
    │   ├── recorder.cpp
    │   ├── res
    │   └── utils
    │       ├── cvtest.cpp
    │       ├── get_pic.cpp
    │       └── test_cam.cpp
    └── ros_related
        ├── pipe_params.yaml
        ├── res
        └── src
            ├── armor_detect
            ├── armor_pipeline
            ├── cv_topic
            └── str_topic
   
```
