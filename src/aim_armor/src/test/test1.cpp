#include <iostream>
#include "openvino/openvino.hpp"

int main() {
	try {
		// 获取并输出 OpenVINO 版本信息
		std::cout << "OpenVINO version: "
		          << ov::get_openvino_version()
		          << std::endl;
	} catch(const std::exception& ex) {
		std::cerr << "Error: " << ex.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
