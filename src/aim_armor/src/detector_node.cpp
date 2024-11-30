#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>
#include <interfaces/msg/armor.hpp>
#include <interfaces/msg/armors.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("detector_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/stream", 10,
            std::bind(&ImageSubscriber::detector, this, std::placeholders::_1)
        );
        
        // 创建 Armors 消息的发布器
        armors_publisher_ = this->create_publisher<interfaces::msg::Armors>(
            "armor_detection", 10
        );
    }

private:
    void detector(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            return;
        }

		// 测试消息填充
        std::vector<interfaces::msg::Armor> armors;
        for (int i = 0; i < 3; ++i) {
            interfaces::msg::Armor armor;
            armor.number = "1";
            armor.type = "BIG";
            armor.distance_to_image_center = 100.0f;

            // 填充 Pose 信息
            armor.pose.position.x = i * 100.0;
            armor.pose.position.y = i * 50.0;
            armor.pose.position.z = 0.0;
            armor.pose.orientation.x = 0.0;
            armor.pose.orientation.y = 0.0;
            armor.pose.orientation.z = 0.0;
            armor.pose.orientation.w = 1.0;

            // 填充关键点信息
            armor.kpts.push_back(geometry_msgs::msg::Point());
            armor.kpts.back().x = i * 10.0;
            armor.kpts.back().y = i * 5.0;
            armor.kpts.back().z = 0.0;

            armors.push_back(armor);
        }

        // 创建 Armors 消息
        interfaces::msg::Armors armors_msg;
        armors_msg.header.stamp = this->get_clock()->now();  // 填充时间戳
        armors_msg.header.frame_id = "camera_frame";  // 填充帧ID
        armors_msg.armors = armors;  // 将检测到的装甲板数组赋值给消息

        // 发布 Armors 消息
        armors_publisher_->publish(armors_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<interfaces::msg::Armors>::SharedPtr armors_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
