cmake_minimum_required(VERSION 3.22)
project(omni_vision LANGUAGES CXX)

# 配置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# -------------------- 依赖查找 --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

# 新增：自定义消息依赖
find_package(omni_interfaces REQUIRED)

# 第三方库依赖
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(fmt REQUIRED)

# -------------------- 包含目录 --------------------
include_directories(
    include
    src
    tasks
    tools
)

# -------------------- 源文件列表 --------------------
set(SOURCES
    src/omni_node.cpp
    tasks/classifier.cpp
    tasks/yolo.cpp
    tasks/yolos/yolov8.cpp
    tasks/detector.cpp
    tasks/armor.cpp
    tools/img_tools.cpp
)

# -------------------- 生成可执行文件 --------------------
add_executable(omni_vision_node ${SOURCES})

# -------------------- 依赖链接 --------------------
ament_target_dependencies(omni_vision_node
    PUBLIC
        rclcpp
        rclcpp_components
        sensor_msgs
        cv_bridge
        std_msgs
        omni_interfaces   # 新增
)

target_link_libraries(omni_vision_node
    PRIVATE
        ${OpenCV_LIBS}
        Eigen3::Eigen
        yaml-cpp
        fmt::fmt
)

if(TARGET openvino::runtime)
    target_link_libraries(omni_vision_node PRIVATE openvino::runtime)
elseif(TARGET openvino)
    target_link_libraries(omni_vision_node PRIVATE openvino)
endif()

# -------------------- 安装规则 --------------------
install(TARGETS omni_vision_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.xml"
)

install(DIRECTORY assets/
    DESTINATION share/${PROJECT_NAME}/assets
)

ament_package()
