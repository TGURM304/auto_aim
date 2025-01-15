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
	toml::table config_file = toml::parse_file("./assets/config.toml");
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
				serial_driver.init_port(serial_port, serial_config);
				serial_driver.port()->open();
				std::cout << serial_port << "已打开" << std::endl;
				return 0;
			} catch(const std::exception& e) {
				std::cerr << "初始化" << serial_port << "失败: " << e.what()
				          << std::endl;
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
			std::cout << "fuck" << ' ' << std::endl;
			for(auto i: _buffer)
				std::cout << (int)i << ' ';
			std::cout << std::endl;
		}
		_buffer.clear();
	}

	return okay;
}
