#include <cstdint>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <interfaces/msg/target.hpp>
#include <interfaces/msg/aim_mode.hpp>
#include <chrono>

#include "serial.hpp"


using TargetMsg = interfaces::msg::Target;
using AimModeMsg = interfaces::msg::AimMode;
using namespace std::chrono_literals;


class SerialReceiver: public rclcpp::Node {
public:
	SerialReceiver(Serial &serial):
	Node("serial_receiver_node"), serial_(serial) {
		aim_mode_msg_.mode = 'a';

		publisher_ = this->create_publisher<AimModeMsg>("/serial/mode", 10);
		timer_ = this->create_wall_timer(
		    1ms, std::bind(&SerialReceiver::reveriver, this));
	}

private:
	void reveriver() {
		ReceiveData data;
		// while(serial_.receiver(data) < 0)
		// 	/* nothing */ std::cout << serial_.receiver(data) << std::endl;
		if(serial_.receiver(data) <= 0) {
			// std::cout << serial_.receiver(data) << std::endl;
			return;
		}
		aim_mode_msg_.color = data.detect_color;
		std::cout << data.detect_color << ' ' << data.pitch << ' ' << data.yaw
		          << std::endl;
		publisher_->publish(aim_mode_msg_);
	}

private:
	Serial &serial_;
	AimModeMsg aim_mode_msg_;

	rclcpp::Publisher<AimModeMsg>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};


class SerialSender: public rclcpp::Node {
public:
	SerialSender(Serial &serial): Node("serial_sender_node"), serial_(serial) {
		timer_ = this->create_wall_timer(
		    2ms, std::bind(&SerialSender::send_data, this));

		subscription_ = this->create_subscription<TargetMsg>(
		    "/target/armor", 10,
		    std::bind(&SerialSender::targetCallback, this,
		              std::placeholders::_1));
	}

private:
	void send_data() {
		serial_.send_target(data_);
	}
	void targetCallback(const TargetMsg::SharedPtr msg) {
		data_.mode = msg->aim_mode;
		data_.pitch_angle = msg->pitch_angle;
		data_.yaw_angle = msg->yaw_angle;
		data_.distance = msg->distance;
	}

	Serial &serial_;
	SendData data_;
	rclcpp::Subscription<TargetMsg>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv) {
	// 初始化串口
	Serial serial;
	serial.init();

	rclcpp::init(argc, argv);

	// 创建节点，并传入 Serial 实例
	auto serial_receive_node = std::make_shared<SerialReceiver>(serial);
	auto serial_send_node = std::make_shared<SerialSender>(serial);

	// 使用多线程执行器来管理节点
	rclcpp::executors::MultiThreadedExecutor executor;
	// 添加节点到执行器
	executor.add_node(serial_receive_node);
	executor.add_node(serial_send_node);
	// 启动执行器
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
