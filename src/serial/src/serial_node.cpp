#include <cstdint>
#include <interfaces/msg/detail/aim_mode__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <interfaces/msg/armors.hpp>
#include <interfaces/msg/aim_mode.hpp>
#include <chrono>

#include "serial.hpp"

using ArmorsMsg = interfaces::msg::Armors;
using AimMode = interfaces::msg::AimMode;

class SerialReceiver: public rclcpp::Node {
public:
	SerialReceiver(): Node("serial_receiver_node") {
		// 创建一个发布者，发布 std_msgs::msg::String 类型的消息
		publisher = this->create_publisher<AimMode>("/serial/mode", 10);

		// 设置定时器，每1秒发布一次消息
		timer = this->create_wall_timer(std::chrono::microseconds(1),std::bind(&SerialReceiver::timer_callback, this));
	}

private:
	void timer_callback() {
		// 创建消息并发布
		auto message = AimMode();
		message.color = 1;
        message.mode = 0;
		publisher -> publish(message);
	}

	Serial::Data data;
	rclcpp::Publisher<AimMode>::SharedPtr publisher;
	rclcpp::TimerBase::SharedPtr timer;
};


class SerialSender: public rclcpp::Node {
public:
	SerialSender(): Node("serial_sender_node") {
    // 订阅
    subscription = this->create_subscription<ArmorsMsg>("/detector/armors", 10, [](const ArmorsMsg::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("armor_subscriber_node"),
                    "Received armors message with %zu armors.", msg->armors.size());


    });
	}

private:
	void callback(const ArmorsMsg::SharedPtr msg) {
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
	}

	rclcpp::Subscription<ArmorsMsg>::SharedPtr subscription;
};



int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto publisher_node = std::make_shared<SerialReceiver>();
	auto subscriber_node = std::make_shared<SerialSender>();

	// 使用多线程执行器来管理节点
	rclcpp::executors::MultiThreadedExecutor executor;
	// 添加节点到执行器
	executor.add_node(publisher_node);
	executor.add_node(subscriber_node);
	// 启动执行器
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
