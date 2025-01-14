#include <cstdint>


/// @brief 发送数据结构体
struct __attribute__((packed)) SendData {
	uint8_t head = 0xFE;
	uint8_t mode = 'a';
	float pitch_angle = 0.0;
	float yaw_angle = 0.0;
	float distance = 0.0;
	uint8_t tail = 0xFF;
};

/// @brief 接收数据结构体
struct __attribute__((packed)) ReceiveData {
	uint8_t header = 0x5A;
	uint8_t detect_color = 'r';
	uint8_t tail = 0xA5;
};
