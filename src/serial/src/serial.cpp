#include <termios.h>
#include <serial.hpp>


Serial::Serial(){
	std::cout<< "Serial()" << std::endl;
};

Serial::~Serial(){
    if (fd != -1) {
        close(fd);
    }
	std::cout<< "~Serial()" << std::endl;
};

int Serial::init(){
	// 读取配置文件
    const char* serial_port = config["serial"]["port"].value_or("/dev/ttyUSB0");
	int in_baud = config["serial"]["in_baud_rate"].value_or(B115200);
	int out_baud = config["serial"]["in_baud_rate"].value_or(B115200);

	// 打开串口
    fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
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
	write(fd, reinterpret_cast<uint8_t*>(&data), sizeof(Data));
}


bool Serial::receiveData(int fd, Data& data) {
    uint8_t buffer[sizeof(Data)];

    // 从串口读取数据
    ssize_t bytes_read = read(fd, buffer, sizeof(Data));
    
    if (bytes_read == -1) {
        return false;
    } else if (bytes_read != sizeof(Data)) {
        return false;
    } else {
        // 将接收到的字节流转换回结构体
        std::memcpy(&data, buffer, sizeof(Data));
		return true;
    }
}