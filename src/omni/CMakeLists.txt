cmake_minimum_required(VERSION 3.22)
project(omni_vision LANGUAGES CXX)

# 配置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 可选：添加编译警告（便于调试）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# -------------------- 依赖查找 --------------------
# ROS2 核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)  # 新增：用于 cv_bridge 的 Header 消息

# 第三方库依赖
find_package(OpenCV REQUIRED)        # OpenCV（图像处理）
find_package(Eigen3 REQUIRED)        # Eigen（矩阵计算）
find_package(yaml-cpp REQUIRED)      # yaml-cpp（配置文件解析）
find_package(OpenVINO REQUIRED)      # OpenVINO（模型推理）
find_package(fmt REQUIRED)           # fmt（日志格式化）

# -------------------- 包含目录 --------------------
# 让编译器能找到项目内的头文件（如 tasks/、tools/ 下的 .hpp）
include_directories(
    include
    src
    tasks        # 包含 detector.hpp、armor.hpp 等
    tools        # 包含 img_tools.hpp 等
)

# -------------------- 源文件列表 --------------------
# 所有需要编译的 .cpp 文件（新增 detector.cpp、armor.cpp）
set(SOURCES
    # 节点主文件
    src/omni_node.cpp
    # 任务模块文件
    tasks/classifier.cpp
    tasks/yolo.cpp
    tasks/yolos/yolov8.cpp
    tasks/detector.cpp       # 新增：Detector 类实现
    tasks/armor.cpp          # 新增：Armor 类实现
    # 工具类文件
    tools/img_tools.cpp
)

# -------------------- 生成可执行文件 --------------------
add_executable(omni_vision_node ${SOURCES})

# -------------------- 依赖链接 --------------------
# 1. ROS2 依赖：用 ament_target_dependencies 自动处理编译/链接选项
ament_target_dependencies(omni_vision_node
    PUBLIC
        rclcpp
        rclcpp_components
        sensor_msgs
        cv_bridge
        std_msgs
)

# 2. 第三方库依赖：用 target_link_libraries 直接链接
target_link_libraries(omni_vision_node
    PRIVATE
        ${OpenCV_LIBS}        # OpenCV（兼容新旧版本）
        Eigen3::Eigen         # Eigen（命名空间格式）
        yaml-cpp              # yaml-cpp（多数版本无命名空间）
        fmt::fmt              # fmt（命名空间格式）
)

# 3. OpenVINO 兼容链接（处理不同版本的目标名称差异）
if(TARGET openvino::runtime)
    target_link_libraries(omni_vision_node PRIVATE openvino::runtime)
elseif(TARGET openvino)
    target_link_libraries(omni_vision_node PRIVATE openvino)
endif()

# -------------------- 安装规则 --------------------
# 1. 安装可执行文件（到 ROS2 包的 lib 目录）
install(TARGETS omni_vision_node
    DESTINATION lib/${PROJECT_NAME}
)

# 2. 安装配置文件（config/ 目录下的 yaml 等）
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.xml"  # 只安装特定格式文件
)

# 3. 安装资源文件（assets/ 目录下的模型、权重等）
install(DIRECTORY assets/
    DESTINATION share/${PROJECT_NAME}/assets
)

# -------------------- ROS2 包声明 --------------------
ament_package()