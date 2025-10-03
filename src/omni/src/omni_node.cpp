#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "rclcpp_components/register_node_macro.hpp"

#include "yolo.hpp"
#include "img_tools.hpp"
#include "armor.hpp"

class OmniNode : public rclcpp::Node
{
public:
    explicit OmniNode(const rclcpp::NodeOptions & options)
    : Node("omni_node", options)
    {
        fov_x_ = this->declare_parameter("fov_x", 70.0);
        fov_y_ = this->declare_parameter("fov_y", 42.0);
        camera_id_ = this->declare_parameter("camera_id", 0);

        // 打开摄像头
        cap_.open(camera_id_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera %d", camera_id_);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Camera %d opened", camera_id_);

        // 发布原始图像
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS());

        // 发布检测结果图像
        result_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/omni/result", rclcpp::SensorDataQoS());

        // YOLO 实例化
        auto config_path = "/home/yeltsa/Desktop/Omni_vision/src/omni/config/omni.yaml";
        yolo_ = std::make_shared<auto_aim::YOLO>(config_path, true);

        // 启动抓帧线程
        std::thread([this]() { captureLoop(); }).detach();
    }

private:
    // ---------------- 成员变量 ----------------
    float fov_x_;
    float fov_y_;
    int camera_id_;
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_pub_;
    std::shared_ptr<auto_aim::YOLO> yolo_;

    struct ArmorAngle
    {
        float yaw;
        float pitch;
    };

    // ---------------- 成员函数 ----------------
    void captureLoop()
    {
        cv::Mat frame;
        rclcpp::Rate rate(30); // 30 FPS

        while (rclcpp::ok()) {
            cap_ >> frame;
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Empty frame captured!");
                rate.sleep();
                continue;
            }

            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            img_pub_->publish(*img_msg);

            // 检测并发布带标注的图像
            publishResult(frame);

            rate.sleep();
        }
    }

    void publishResult(const cv::Mat & frame)
    {
        cv::Mat img_with_boxes = frame.clone();
        auto armors = yolo_->detect(frame);

        for (const auto & armor : armors) {
            // 绘制装甲板矩形
            for (size_t i = 0; i < 4; i++) {
                cv::line(img_with_boxes, armor.points[i], armor.points[(i + 1) % 4],
                         cv::Scalar(0, 255, 0), 2);
            }
            // 绘制中心点
            cv::circle(img_with_boxes, armor.center, 3, cv::Scalar(0, 0, 255), -1);
        }

        auto result_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_with_boxes).toImageMsg();
        result_pub_->publish(*result_msg);
    }
};

// 注册组件节点
RCLCPP_COMPONENTS_REGISTER_NODE(OmniNode)

// main 函数（独立可执行文件入口）
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
