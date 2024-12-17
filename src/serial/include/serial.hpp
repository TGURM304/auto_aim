#ifndef _SERIAL_H_SERIAL_
#define _SERIAL_H_SERIAL_

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <toml.hpp>

class Serial {
public:
	Serial();

	~Serial();

	/// @brief 串口数据结构体
	struct __attribute__((packed)) Data {
		uint8_t head = 0x7E;
		uint8_t mode = 0;
		float pitch_angle = 0.0;
		float yaw_angle = 0.0;
		uint8_t tail = 0x7F;
	};

	/**
     * @brief 初始化串口设备
     * 
     * @return 1 初始化成功
     * @return -1 打开串口失败
     * @return -2 获取串口配置失败
     * @return -3 应用串口配置失败
     */
	int init();

	/**
     * @brief 发送串口数据
     * 
     * @param fd file descriptor(文件描述符)
     * @param data 数据结构体
     */
	void sendData(Data& data);

	/**
     * @brief 接受串口数据
     * 
     * @param fd file descriptor(文件描述符)
     * @param data 数据结构体
     * @return true 
     * @return false 
     */
	bool receiveData(Data& data);

private:
	/// @brief 初始化配置文件
	toml::table config = toml::parse_file("./assets/config.toml");
	/// @brief 串口配置
	struct termios options;
	/// @brief 串口文件描述符
	int fd;
};

#endif