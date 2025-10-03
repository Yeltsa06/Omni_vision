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
        // 声明参数
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

        // 发布器
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS());

        // 订阅器
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&OmniNode::imageCallback, this, std::placeholders::_1));

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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
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

            rate.sleep();
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        auto img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        auto armors = yolo_->detect(img);  // 注意这里用 ->

        if (armors.empty()) return;

        cv::Point2f img_center(img.cols / 2.0f, img.rows / 2.0f);

        auto closest_it = std::min_element(
            armors.begin(), armors.end(),
            [&img_center](const auto_aim::Armor & a, const auto_aim::Armor & b) {
                float da = cv::norm(a.center - img_center);
                float db = cv::norm(b.center - img_center);
                return da < db;
            });

        auto_aim::Armor closest_armor = *closest_it;
        armors.clear();
        armors.push_back(closest_armor);

        ArmorAngle angle = computeArmorAngle(closest_armor, img.cols, img.rows);
        RCLCPP_INFO(this->get_logger(), "Closest Armor: yaw=%.2f, pitch=%.2f", angle.yaw, angle.pitch);
    }

    ArmorAngle computeArmorAngle(const auto_aim::Armor & armor, int img_width, int img_height)
    {
        cv::Point2f img_center(img_width / 2.0f, img_height / 2.0f);
        cv::Point2f offset = armor.center - img_center;

        ArmorAngle angle;
        angle.yaw   = (offset.x / img_width) * fov_x_;
        angle.pitch = -(offset.y / img_height) * fov_y_;
        return angle;
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
