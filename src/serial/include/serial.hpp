#include <serial_driver/serial_driver.hpp>
#include <iostream>
#include "package.hpp"
#include "toml.hpp"


using namespace drivers::serial_driver;
using namespace drivers::common;


class Serial {

public:
	Serial():
	io_context(2),
	serial_driver(io_context) {}

	~Serial() {
		if(serial_driver.port()->is_open()) {
			serial_driver.port()->close();
		}
	}
	/**
     * @brief 串口初始化
     * 
     * @return 0为正常，1为初始化失败
     */
	int init();


	size_t send_target(Data4Send &data);

	size_t receiver(Data4Receive &data);

private:
	std::shared_ptr<SerialPort> port;
	IoContext io_context;
	SerialDriver serial_driver;
};