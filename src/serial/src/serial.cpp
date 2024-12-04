#include <termios.h>
#include <serial.hpp>


struct __attribute__((packed)) Serial::Data{
	uint8_t mode;
	float pitch_angle;
	float yaw_angle;
};

Serial::Serial(){
	std::cout<< "Serial()" << std::endl;
};

Serial::~Serial(){
	std::cout<< "~Serial()" << std::endl;
};

int Serial::init(){
	// 读取配置文件
    const char* serial_port = config["serial"]["port"].value_or("/dev/ttyUSB0");
	int in_baud = config["serial"]["in_baud_rate"].value_or(B115200);
	int out_baud = config["serial"]["in_baud_rate"].value_or(B115200);

	// 打开串口
    int fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("打开串口失败");
        return -1;
    }

	if (tcgetattr(fd, &options) < 0) {
        perror("获取串口配置失败");
        close(fd);
        return -2;
    }

	// 设置输入输出波特率
	cfsetispeed(&options, in_baud);
	cfsetospeed(&options, out_baud);

	// 设置数据位、停止位、校验位
    options.c_cflag &= ~PARENB;    // 无校验位
    options.c_cflag &= ~CSTOPB;    // 1个停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;        // 8个数据位
    options.c_cflag |= CLOCAL | CREAD;  // 使能接收，忽略调制解调器状态线
    options.c_cc[VMIN] = 1;  // 至少读取1个字节
    options.c_cc[VTIME] = 0; // 设置超时

	// 应用设置
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("应用串口配置失败");
        close(fd);
        return -3;
    }

    return 1;
}

void Serial::sendData(int fd, Data& data) {
    uint8_t frame[sizeof(Data) + 2];

    // 帧头
    frame[0] = 0x7E;

    // 数据
    std::memcpy(&frame[1], &data, sizeof(Data));

    // 帧尾
    frame[sizeof(Data) + 1] = 0x7F;

    // 发送数据
    write(fd, frame, sizeof(frame));
}


bool Serial::receiveData(int fd, Data& data) {
    uint8_t frame[sizeof(Data) + 2];

    // 读取数据
    read(fd, frame, sizeof(frame));

    // 校验帧头和帧尾
    if (frame[0] == 0x7E && frame[sizeof(Data) + 1] == 0x7F) {
        std::memcpy(&data, &frame[1], sizeof(Data));
		return true;
    } else {
        std::cerr << "error" << std::endl;
		return false;
    }
}