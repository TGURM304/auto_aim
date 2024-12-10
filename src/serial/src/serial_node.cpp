#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/armors.hpp>
#include <std_msgs/msg/string.hpp>

using ArmorsMsg = interfaces::msg::Armors;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("serial_node");

    // 订阅
    auto subscription = node->create_subscription<ArmorsMsg>("/detector/armors", 10, [](const ArmorsMsg::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("armor_subscriber_node"),
                    "Received armors message with %zu armors.", msg->armors.size());

        // test
        for (const auto& armor : msg->armors) {
            RCLCPP_INFO(rclcpp::get_logger("serial_node"),
                        "Armor number: %s, Type: %s, Distance to center: %.2f",
                        armor.number.c_str(), armor.type.c_str(), armor.distance_to_image_center);

            for (const auto& kpt : armor.kpts) {
                RCLCPP_INFO(rclcpp::get_logger("serial_node"),
                            "Keypoint - x: %.2f, y: %.2f, z: %.2f", kpt.x, kpt.y, kpt.z);
            }
        }
    });

    // 发布
    auto publisher = node->create_publisher<std_msgs::msg::String>("/serial/receive", 10);
    auto timer = node->create_wall_timer(std::chrono::microseconds(1),[publisher]() {
            auto message = std_msgs::msg::String();
            message.data = "Hello from serial_node!";
            publisher->publish(message);
            RCLCPP_INFO(rclcpp::get_logger("serial_node"), "Published: %s", message.data.c_str());
        });

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
