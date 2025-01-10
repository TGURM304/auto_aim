#ifndef _SERIAL_H_SERIAL_
#define _SERIAL_H_SERIAL_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <rclcpp/utilities.hpp>
#include <sys/types.h> 
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h> 
#include <stdint.h>

#include "toml.hpp"

class Serial {
public:
	Serial();

	~Serial();

	/// @brief 发送数据结构体
	struct __attribute__((packed)) Send_Data {
		uint8_t head = 0xFE;
		uint8_t mode = 0;
		float pitch_angle = 0.0;
		float yaw_angle = 0.0;
		float distance = 0.0;
		uint8_t tail = 0xFF;
	};

	/// @brief 接收数据结构体
	struct __attribute__((packed)) Receive_Data {
        uint8_t header = 0x5A;
        uint8_t detect_color = 'r';
        uint8_t tail = 0xA5;
	};

	/**
	 * @brief 初始化串口设备
	 *
	 * @return 0 初始化成功
	 * @return -1 打开串口失败
	 * @return -2 获取串口配置失败
	 * @return -3 应用串口配置失败
	 */
	int init();

	/**
	 * @brief 发送串口数据
	 *
	 * @param fd file descriptor (文件描述符)
	 * @param data 数据结构体
	 */
	void sendData(Send_Data& data);

	/**
	 * @brief 接受串口数据
	 *
	 * @param fd file descriptor (文件描述符)
	 * @param data 数据结构体
	 */
	bool receiveData(Receive_Data& data);

private:
	/// @brief 串口配置
	struct termios options;
	/// @brief 串口文件描述符
	int fd;
};

#endif
