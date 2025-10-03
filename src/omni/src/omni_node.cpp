#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "rclcpp_components/register_node_macro.hpp"

#include "yolo.hpp"
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

        cap_.open(camera_id_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera %d", camera_id_);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Camera %d opened", camera_id_);

        // 发布检测结果
        result_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/omni_result", rclcpp::SensorDataQoS());

        auto config_path = "/home/yeltsa/Desktop/Omni_vision/src/omni/config/omni.yaml";
        yolo_ = std::make_shared<auto_aim::YOLO>(config_path, false);

        // 启动采集和推理循环
        std::thread([this]() { captureLoop(); }).detach();
    }

private:
    float fov_x_;
    float fov_y_;
    int camera_id_;
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_pub_;
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

                RCLCPP_INFO(this->get_logger(), 
                    "Closest Armor: yaw=%.2f, pitch=%.2f", angle.yaw, angle.pitch);

                // 绘制结果
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
        angle.yaw   = (offset.x / img_width) * fov_x_;
        angle.pitch = -(offset.y / img_height) * fov_y_;
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
