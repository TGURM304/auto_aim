#include <serial_driver/serial_driver.hpp>
#include <iostream>

#include "package.hpp"
#include "toml.hpp"


using namespace drivers::serial_driver;
using namespace drivers::common;


class Serial {
public:
	Serial(): io_context(2), serial_driver(io_context) {}

	~Serial() {
		if(serial_driver.port()->is_open()) {
			serial_driver.port()->close();
		}
	}

	/**
	 * @brief 串口初始化
	 * 
	 * @warning 当初始化失败时, 将自动重新初始化.
	 *          可能导致阻塞.
	 */
	int init();

	/**
	 * @brief 发送一条数据
	 */
	size_t send_target(const SendData &data);

	/**
	 * @brief 接收数据, 存到 `data` 里
	 * 
	 * @return    返回收到的数据长度.
	 *            若接受格式不对, 返回错误码 (小于 0).
	 * @retval -1 接收到的数据长度为 0
	 * @retval -2 校验位 (头尾) 不对
	 * @retval -3 内部数据 (除去校验位) 长度不对
	 */
	int receiver(ReceiveData &data);

private:
	std::shared_ptr<SerialPort> port;
	IoContext io_context;
	SerialDriver serial_driver;
};
