#include <chrono>
#include <functional>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

#include "toml.hpp"
#include "crc.hpp"
#include "package.hpp"
#include "serial_node.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace drivers::serial_driver;
using namespace drivers::common;


std::list<std::string> expand_ports(const std::string& port_pattern) {
	std::list<std::string> expanded_ports;

	if(port_pattern.find('*') != std::string::npos) {
		std::string prefix = port_pattern.substr(0, port_pattern.find('*'));
		std::string suffix = port_pattern.substr(port_pattern.find('*') + 1);
		for(int i = 0; i <= 5; ++i) {
			std::stringstream ss;
			ss << prefix << i << suffix;
			expanded_ports.push_back(ss.str());
		}
	} else {
		expanded_ports.push_back(port_pattern);
	}

	return expanded_ports;
}


SerialNode::SerialNode():
Node("serial_node"), io_context_(2), serial_driver_(io_context_) {
	sub_ = this->create_subscription<TargetMsg>(
	    "/target/armor", 10, std::bind(&SerialNode::send_callback, this, _1));
	pub_ = this->create_publisher<AimModeMsg>("/serial/mode", 10);
}

SerialNode::~SerialNode() {
	if(receive_thread_->joinable()) {
		receive_thread_->join();
	}

	delete receive_thread_;

	if(serial_driver_.port()->is_open()) {
		serial_driver_.port()->close();
	}
}

int SerialNode::init() {
	// FIXME: 配置导入
	toml::table config_file = toml::parse_file("assets/config.toml");
	auto ports = config_file["serial"]["port"].as_array();
	int baud_rate = config_file["serial"]["baud_rate"].value_or(B115200);
	SerialPortConfig serial_config(baud_rate, FlowControl::NONE, Parity::NONE,
	                               StopBits::ONE);

	std::list<std::string> portlist;
	for(const auto& port: *ports) {
		auto expanded_ports = expand_ports(port.as_string()->get());
		portlist.splice(portlist.end(), expanded_ports);
	}

	while(1) {
		for(auto& serial_port: portlist) {
			try {
				serial_driver_.init_port(serial_port, serial_config);
				serial_driver_.port()->open();
				receive_thread_ = new std::thread(
				    std::bind(&SerialNode::receive_process, this));
				RCLCPP_INFO(this->get_logger(), "%s is opened",
				            serial_port.c_str());
				return 0;
			} catch(const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "%s fail to open; %s",
				             serial_port.c_str(), e.what());
			}
		}
	}
	return 0;
}

void SerialNode::receive_process() {
	std::vector<uint8_t> header(1);
	std::vector<uint8_t> buffer;
	buffer.reserve(sizeof(ReceiveData));
	ReceiveData data;

	while(true) {
		try {
			serial_driver_.port()->receive(header);

			if(header[0] == 0x5A) {
				buffer.resize(sizeof(ReceiveData) - 1);
				serial_driver_.port()->receive(buffer);

				buffer.insert(buffer.begin(), header[0]);
				std::copy(buffer.begin(), buffer.end(),
				          reinterpret_cast<uint8_t*>(&data));

				bool crc_ok = CRC16::verify(data);
				if(crc_ok) {
					AimModeMsg mode;
					mode.mode = 'a';
					mode.color = data.detect_color;
					mode.pitch = data.pitch;
					mode.yaw = data.yaw;
					pub_->publish(mode);
				} else {
					RCLCPP_ERROR(this->get_logger(), "CRC error!");
				}
			} else {
				RCLCPP_WARN(this->get_logger(), "Invalid header: %02X",
				            header[0]);
			}
		} catch(const std::exception& ex) {
			RCLCPP_ERROR(this->get_logger(), "Error while receiving buffer: %s",
			             ex.what());
			RCLCPP_WARN(this->get_logger(), "Attempting to reopen port");
			while(true) {
				try {
					if(serial_driver_.port()->is_open()) {
						serial_driver_.port()->close();
					}
					serial_driver_.port()->open();
					RCLCPP_INFO(this->get_logger(),
					            "Successfully reopened port");
					break;
				} catch(const std::exception& ex) {
					RCLCPP_ERROR(this->get_logger(),
					             "Error while reopening port: %s", ex.what());
				}
			}
		}
	}
}

void SerialNode::send_callback(const TargetMsg::SharedPtr msg) {
	SendData data;
	size_t buffer_size = sizeof(data);

	data.mode = msg->aim_mode;
	data.pitch_angle = msg->pitch_angle;
	data.yaw_angle = msg->yaw_angle;
	data.distance = msg->distance;
	CRC16::append(data);

	std::vector<uint8_t> buffer(buffer_size);
	std::memcpy(buffer.data(), &data, buffer_size);
	serial_driver_.port()->send(buffer);
}


int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto serial_node = std::make_shared<SerialNode>();
	serial_node->init();
	rclcpp::spin(serial_node);

	rclcpp::shutdown();
	return 0;
}
