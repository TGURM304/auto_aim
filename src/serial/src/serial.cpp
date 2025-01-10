#include <serial.hpp>


Serial::Serial() {
	std::cout << "Serial()" << std::endl;
};

Serial::~Serial() {
	if(fd != -1) {
		close(fd);
	}
	std::cout << "~Serial()" << std::endl;
};

int Serial::init() {
	// 读取配置文件
	toml::table config = toml::parse_file("./assets/config.toml");
	const char* serial_port = config["serial"]["port"].value_or("/dev/ttyUSB0");
	int in_baud = config["serial"]["in_baud_rate"].value_or(B2000000);
	int out_baud = config["serial"]["out_baud_rate"].value_or(B2000000);

	std::cout << serial_port << std::endl;

	// 打开串口
	fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1) {
		perror("打开串口失败");
		return -1;
	}

	system("stty -F /dev/ttyCH341USB1 2000000 cs8 -cstopb -parenb");
	// 获取当前串口设置
	tcgetattr(fd, &options);


	// 设置波特率
	cfsetispeed(&options, in_baud);
	cfsetospeed(&options, out_baud);

	// 配置串口数据位、停止位、校验位
	options.c_cflag &= ~PARENB;  // 无奇偶校验
	options.c_cflag &= ~CSTOPB;  // 一个停止位
	options.c_cflag &= ~CSIZE;   // 清除数据位大小掩码
	options.c_cflag |= CS8;      // 8 数据位
	options.c_cflag &= ~CRTSCTS; // 禁用硬件流控
	options.c_cflag |= CREAD | CLOCAL; // 启用接收器，并且忽略 modem 控制线
	options.c_iflag &= ~ICANON; // 禁用规范模式
	options.c_iflag &= ~ECHO;   // 禁用回显
	options.c_iflag &= ~ECHOE;  // 禁用回显擦除
	options.c_iflag &= ~ISIG;   // 禁用信号处理

	options.c_oflag &= ~OPOST; // 禁用输出处理

	// 设置最小读取字符数和读取超时
	options.c_cc[VMIN] = 1;  // 至少读取一个字符
	options.c_cc[VTIME] = 0; // 无超时

	// 应用串口配置
	tcsetattr(fd, TCSANOW, &options);


	tcflush(fd, TCIFLUSH); // 清空串口的输入缓冲区

	return 0;
}


void Serial::sendData(Send_Data& data) {
	// std::cout << data.mode << std::endl;
	// std::cout << data.pitch_angle << std::endl;
	// std::cout << data.yaw_angle << std::endl;
	// std::cout << data.distance << std::endl;
	// std::cout << "----------" << std::endl;
	write(fd, reinterpret_cast<uint8_t*>(&data), sizeof(data));
}

bool Serial::receiveData(Receive_Data& data) {
	uint8_t buffer[sizeof(data)];
	while(rclcpp::ok()) {
		ssize_t bytes_read = read(fd, buffer, sizeof(data));
		if(buffer[0] == data.header && buffer[sizeof(data) - 1] == data.tail) {
			// if(buffer[0] == 0x7E && buffer[sizeof(data)-1] == 0x7F) {
			for(size_t i = 0; i < sizeof(data); ++i) {
				printf("%02X ", buffer[i]);
			}
			std::cout << std::endl;
			// 将接收到的字节流转换回结构体
			std::memcpy(&data, buffer, sizeof(data));
			return true;
		}
	}
}
