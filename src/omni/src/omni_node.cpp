#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "rclcpp_components/register_node_macro.hpp"

#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

class OmniNode : public rclcpp::Node
{
public:
    explicit OmniNode(const rclcpp::NodeOptions & options)
    : Node("omni_node", options)
    {
        // 读取 FOV 参数（可从 YAML 配置文件传入）
        fov_x_ = this->declare_parameter("fov_x", 70.0);  // 默认水平视场 70 度
        fov_y_ = this->declare_parameter("fov_y", 42.0);  // 默认垂直视场 42 度

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

        // 订阅器（同一节点也可以订阅自己发布的图像）
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&OmniNode::imageCallback, this, std::placeholders::_1)
        );

        // 启动抓帧线程
        std::thread([this]() { captureLoop(); }).detach();
        
        // 类实例化
        auto config_path = "/config/omni.yaml"; // 报错就用绝对路径
        yolo_ = std::make_shared<auto_aim::YOLO>(config_path, false); // 改为true进入debug模式，启用imshow(result)
    }

private:
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

            // 转成 ROS 消息并发布
            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            img_pub_->publish(*img_msg);

            rate.sleep();
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        RCLCPP_INFO(this->get_logger(), "ImageCallback triggered! size=%dx%d",
                    img_msg->width, img_msg->height);
        // TODO: 在这里处理你的图像逻辑

        //yolo检测
        auto img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        auto armors = yolo_.detect(img);
        if (armors.empty()) return;

        // 计算中心
        cv::Point2f img_center(img.cols / 2.0f, img.rows / 2.0f);

        // 找到距离中心最近的装甲板迭代器
        auto closest_it = std::min_element(
            armors.begin(), armors.end(),
            [&img_center](const Armor & a, const Armor & b) {
                float da = cv::norm(a.center - img_center);
                float db = cv::norm(b.center - img_center);
                return da < db;
            });

        // 只保留这个最近的装甲板
        Armor closest_armor = *closest_it;
        armors.clear();
        armors.push_back(closest_armor); // 我猜从这里开始就会报错嘿嘿

        // 计算角度
        ArmorAngle angle = computeArmorAngle(closest_armor, img.cols, img.rows);


        
    }

    struct ArmorAngle
    {
        float yaw;
        float pitch;
    };

    ArmorAngle computeArmorAngle(const auto_aim::Armor & armor, int img_width, int img_height)
    {
        cv::Point2f img_center(img_width / 2.0f, img_height / 2.0f);
        cv::Point2f offset = armor.center - img_center;

        ArmorAngle angle;
        angle.yaw   = (offset.x / img_width) * fov_x_;
        angle.pitch = -(offset.y / img_height) * fov_y_;
        return angle;
    }

    int camera_id_;
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    std::shared_ptr<auto_aim::YOLO> yolo_;
};

// 注册组件节点
RCLCPP_COMPONENTS_REGISTER_NODE(OmniNode)
