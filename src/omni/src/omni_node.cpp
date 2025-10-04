#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "rclcpp_components/register_node_macro.hpp"

#include "yolo.hpp"
#include "armor.hpp"
#include "omni_interfaces/msg/omni_angle.hpp"   // 引入自定义消息
#include <yaml-cpp/yaml.h>
#include <fstream>

class OmniNode : public rclcpp::Node
{
public:
    explicit OmniNode(const rclcpp::NodeOptions & options)
    : Node("omni_node", options)
    {
        // -------------------- 读取参数 --------------------
        auto config_path = this->declare_parameter<std::string>("config_path",
            "/home/yeltsa/Desktop/Omni_vision/src/omni/config/omni.yaml");

        YAML::Node config = YAML::LoadFile(config_path);

        // 敌方颜色（未使用，可留作扩展）
        enemy_color_ = config["enemy_color"].as<std::string>("blue");

        // 相机参数
        camera_id_ = config["camera"]["id"].as<int>(0);
        fov_x_ = config["camera"]["fov_x"].as<float>(70.0);
        fov_y_ = config["camera"]["fov_y"].as<float>(42.0);
        cam_yaw_offset_ = config["camera"]["cam_yaw_offset"].as<float>(180.0);

        // -------------------- 打开相机 --------------------
        cap_.open(camera_id_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera %d", camera_id_);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Camera %d opened", camera_id_);

        // 发布检测结果图片
        result_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/omni_result", rclcpp::SensorDataQoS());

        // 发布角度
        angle_pub_ = this->create_publisher<omni_interfaces::msg::OmniAngle>(
            "/omni_angle", 10);

        // -------------------- YOLO 初始化 --------------------
        yolo_ = std::make_shared<auto_aim::YOLO>(config_path, false);

        // -------------------- 启动采集和推理循环 --------------------
        std::thread([this]() { captureLoop(); }).detach();
    }

private:
    // -------------------- 参数 --------------------
    float fov_x_;
    float fov_y_;
    float cam_yaw_offset_;
    int camera_id_;
    std::string enemy_color_;

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_pub_;
    rclcpp::Publisher<omni_interfaces::msg::OmniAngle>::SharedPtr angle_pub_;
    std::shared_ptr<auto_aim::YOLO> yolo_;

    struct ArmorAngle { float yaw; float pitch; };

    void captureLoop()
    {
        cv::Mat frame;
        rclcpp::Rate rate(30);

        while (rclcpp::ok()) {
            cap_ >> frame;
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Empty frame captured!");
                rate.sleep();
                continue;
            }

            // ---------- YOLO 推理 ----------
            auto armors = yolo_->detect(frame);
            if (!armors.empty()) {
                cv::Point2f img_center(frame.cols / 2.0f, frame.rows / 2.0f);

                auto closest_it = std::min_element(
                    armors.begin(), armors.end(),
                    [&img_center](const auto_aim::Armor & a, const auto_aim::Armor & b) {
                        float da = cv::norm(a.center - img_center);
                        float db = cv::norm(b.center - img_center);
                        return da < db;
                    });

                auto_aim::Armor closest_armor = *closest_it;
                ArmorAngle angle = computeArmorAngle(closest_armor, frame.cols, frame.rows);

                // -------------------- 发布角度 --------------------
                omni_interfaces::msg::OmniAngle angle_msg;
                angle_msg.yaw = angle.yaw;       // 相对于云台正前方
                angle_msg.pitch = angle.pitch;
                angle_pub_->publish(angle_msg);

                RCLCPP_INFO(this->get_logger(), 
                    "Closest Armor: yaw=%.2f, pitch=%.2f", angle.yaw, angle.pitch);

                // -------------------- 绘制结果 --------------------
                cv::Mat img_with_boxes = frame.clone();
                for (size_t i = 0; i < 4; i++) {
                    cv::line(img_with_boxes, closest_armor.points[i], 
                             closest_armor.points[(i + 1) % 4],
                             cv::Scalar(0, 255, 0), 2);
                }
                cv::circle(img_with_boxes, closest_armor.center, 3, cv::Scalar(0, 0, 255), -1);

                auto result_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_with_boxes).toImageMsg();
                result_pub_->publish(*result_msg);
            }
            // -------------------------------------------

            rate.sleep();
        }
    }

    ArmorAngle computeArmorAngle(const auto_aim::Armor & armor, int img_width, int img_height)
    {
        cv::Point2f img_center(img_width / 2.0f, img_height / 2.0f);
        cv::Point2f offset = armor.center - img_center;

        ArmorAngle angle;
        // 计算目标在相机视野内的yaw和pitch
        float yaw_in_cam = (offset.x / img_width) * fov_x_;
        float pitch = -(offset.y / img_height) * fov_y_;

        // 转换为相对于云台正前方的角度
        angle.yaw = yaw_in_cam + cam_yaw_offset_;
        angle.pitch = pitch;
        return angle;
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(OmniNode)

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
