cmake_minimum_required(VERSION 3.22)
project(omni_vision LANGUAGES CXX)

# 使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# OpenVINO
find_package(OpenVINO REQUIRED)

# include 目录
include_directories(
    include
    src
    tasks
    tools
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

# 源文件列表
set(SOURCES
    src/omni_node.cpp
    tasks/classifier.cpp
    tasks/yolo.cpp
    tasks/yolos/yolov8.cpp
    tools/img_tools.cpp
)

# 头文件
set(HEADERS
    tasks/armor.hpp
    tasks/classifier.hpp
    tasks/detector.hpp
    tasks/yolo.hpp
    tasks/yolos/yolov8.hpp
    tools/img_tools.hpp
)

# 可执行文件
add_executable(${PROJECT_NAME}_node ${SOURCES} ${HEADERS})

# 链接库
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    sensor_msgs
    OpenCV
    Eigen3
    yaml-cpp
)

# OpenVINO 库链接
target_link_libraries(${PROJECT_NAME}_node PRIVATE openvino::openvino)

# 安装
install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 config、assets
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY assets/ DESTINATION share/${PROJECT_NAME}/assets)

ament_package()
