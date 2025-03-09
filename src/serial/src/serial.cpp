#include <cstdint>
#include <regex>
#include <string>
#include <list>

#include "serial.hpp"


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

int Serial::init() {
    std::vector<std::string> ports;
    this->declare_parameter("serial.port", ports);  
    int baud_rate = this->declare_parameter("serial.baud_rate", B115200);  
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
				serial_driver.init_port(serial_port, serial_config);
				serial_driver.port()->open();
				RCLCPP_INFO(this->get_logger(), "%s已打开\n", serial_port.c_str());
				return 0;
			} catch(const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "初始化%s失败：%s\n", serial_port.c_str()，e.what());
			}
		}
	}
}

size_t Serial::send_target(const SendData& data) {
	size_t data_size = sizeof(data);

	std::vector<uint8_t> buffer(data_size);
	std::memcpy(buffer.data(), &data, data_size);

	return serial_driver.port()->send(buffer);
}

int Serial::receiver(ReceiveData& data) {
	std::vector<uint8_t> buffer(1);
	if(_buffer.empty()) {
		if(serial_driver.port()->receive(buffer) and buffer[0] == data.header) {
			_buffer.emplace_back(buffer[0]);
			buffer.resize(sizeof(data) - 1);
			size_t sz = serial_driver.port()->receive(buffer);
			for(int i = 0; i < (int)sz; i++)
				_buffer.emplace_back(buffer[i]);
		} else {
			return -1;
		}
	} else {
		buffer.resize(sizeof(data) - _buffer.size());
		size_t sz = serial_driver.port()->receive(buffer);
		for(int i = 0; i < (int)sz; i++)
			_buffer.emplace_back(buffer[i]);
	}

	bool okay = false;

	if(_buffer.size() == sizeof(data)) {
		if(_buffer.back() == data.tail)
			std::memcpy(&data, _buffer.data(), sizeof(data)), okay = true;
		else {
			// 太不雅了
			RCLCPP_INFO(this->get_logger(), "fuck\n");
			for(auto i: _buffer)
				RCLCPP_INFO(this->get_logger(), "%d ",(int)i);
			RCLCPP_INFO(this->get_logger(), "\n");
		}
		_buffer.clear();
	}

	return okay;
}
