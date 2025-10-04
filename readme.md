# 哨兵全向感知测试

本项目为哨兵全向感知测试，旨在实现对目标装甲板的全向检测与角度计算，并通过串口进行数据通信。

## 已实现功能

- YOLOv8 目标识别
- 返回单个装甲板相对角度
- 通过 ROS2 topic 发布检测结果

## 待完成功能

### omni_node
- 订阅 `/tracker_state` 话题，若 tracker 状态为 `lost` 时开始运行
- 多相机切换检测，未识别到目标时自动切换相机

### serial_driver
- 订阅角度消息进行回调
- 打包角度消息
- 新建协议 `0xA7` 用于全向感知串口发布

## 参数说明

| 参数名               | 类型   | 默认值 | 说明                                         |
|--------------------|------|------|--------------------------------------------|
| `camera.id`         | int  | 0    | 相机设备ID                                   |
| `camera.fov_x`      | float| 70.0 | 相机水平视场角 (度)                          |
| `camera.fov_y`      | float| 42.0 | 相机垂直视场角 (度)                          |
| `camera.cam_yaw_offset` | float| 180.0 | 相机相对云台正前方的 yaw 偏移角 (度)       |
| `enemy_color`       | string | "blue" | 识别敌方颜色                                 |
| `yolo_name`         | string | "yolov8" | YOLO 模型类型                              |
| `classify_model`    | string | 路径 | 分类模型 ONNX 文件路径                         |
| `yolov8_model_path` | string | 路径 | YOLOv8 模型 XML 文件路径                       |
| `device`            | string | "CPU" | 推理设备                                     |
| `threshold`         | float  | 0.5  | 检测阈值                                     |

## 主要 ROS2 Topics

| Topic                  | 消息类型                        | 说明                         |
|------------------------|--------------------------------|-----------------------------|
| `/omni_result`         | `sensor_msgs/msg/Image`        | 发布检测结果图像               |
| `/omni_angle`          | `omni_interfaces/msg/OmniAngle` | 发布目标相对云台的 yaw/pitch 角度 |
| `/tracker_state`       | `std_msgs/msg/String`           | 订阅目标追踪状态               |

## 使用方法

1. 编译工作区
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
