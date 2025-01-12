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

class SerialReceiver: public rclcpp::Node {
public:
	SerialReceiver(Serial &serial):
	Node("serial_receiver_node"), serial_(serial) {
		publisher = this->create_publisher<AimModeMsg>("/serial/mode", 10);
		timer = this->create_wall_timer(
		    std::chrono::microseconds(10),
		    std::bind(&SerialReceiver::reveriver, this));
	}

private:
	void reveriver() {
		do {
			receive_size = serial_.receiver(rdata);
		} while(!receive_size);
		std::cout << rdata.detect_color << std::endl;
		aimmodemsg.color = rdata.detect_color;
		publisher->publish(aimmodemsg);
	}

	Serial &serial_;
	AimModeMsg aimmodemsg;
	size_t receive_size;
	Data4Receive rdata;
	rclcpp::Publisher<AimModeMsg>::SharedPtr publisher;
	rclcpp::TimerBase::SharedPtr timer;
};

class SerialSender: public rclcpp::Node {
public:
	SerialSender(Serial &serial): Node("serial_sender_node"), serial_(serial) {

		timer_ =
		    this->create_wall_timer(std::chrono::milliseconds(1),
		                            std::bind(&SerialSender::sendData, this));

		subscription_ = this->create_subscription<TargetMsg>(
		    "/target/armor", 10,
		    std::bind(&SerialSender::targetCallback, this,
		              std::placeholders::_1));
	}

private:
	void sendData() {
		serial_.send_target(sdata);
	}
	void targetCallback(const TargetMsg::SharedPtr msg) {
		sdata.mode = msg->aim_mode;
		sdata.pitch_angle = msg->pitch_angle;
		sdata.yaw_angle = msg->yaw_angle;
		sdata.distance = msg->distance;
	}

	Serial &serial_;
	Data4Send sdata;
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
