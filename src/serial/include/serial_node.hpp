#ifndef _SERIAL_H_SERIAL_NODE_
#define _SERIAL_H_SERIAL_NODE_

#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/target.hpp>
#include <interfaces/msg/aim_mode.hpp>
#include <serial_driver/serial_driver.hpp>

#include "crc.hpp"

using TargetMsg = interfaces::msg::Target;
using AimModeMsg = interfaces::msg::AimMode;


class SerialNode: public rclcpp::Node {
public:
	SerialNode();
	~SerialNode();

	int init();

private:
	void receive_process();
	void send_callback(const TargetMsg::SharedPtr msg);

private:
	rclcpp::Subscription<TargetMsg>::SharedPtr sub_;
	rclcpp::Publisher<AimModeMsg>::SharedPtr pub_;

	std::thread* receive_thread_ = nullptr;

private:
	IoContext io_context_;
	drivers::serial_driver::SerialDriver serial_driver_;
};

#endif
