#include <opencv2/highgui.hpp>
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm> // std::max_element, std::distance

int main() {
	try {
		ov::Core core;

		std::string model_path = "./model/best.onnx";            // 模型路径
		std::string image_path = "./2.png"; // 图像路径
		std::string device_name = "CPU"; // 推理设备（CPU）

		// 加载模型
		auto model = core.read_model(model_path);


		cv::Mat image =
		    cv::imread(image_path); // 使用 OpenCV 读取图像
		if(image.empty()) {
			std::cerr << "无法读取图像: " << image_path
			          << std::endl;
			return -1;
		}

		cv::Mat resized_image;
		cv::resize(image, resized_image, cv::Size(64, 64));

		// 将图像数据转换为浮动精度类型 (f32)，并归一化 [0, 255] -> [0, 1]
		resized_image.convertTo(
		    resized_image, CV_32F,
		    1.0 / 255.0); // 转为 f32 类型并归一化

		ov::Shape input_shape = {
		    1, 3, 64, 64}; // 假设模型输入为 64x64 的图像
		ov::Tensor input_tensor(
		    ov::element::f32, input_shape,
		    resized_image.data); // 使用 f32 类型

		auto compiled_model = core.compile_model(
		    model, device_name); // 编译模型
		ov::InferRequest infer_request =
		    compiled_model
		        .create_infer_request(); // 创建推理请求

		// 设置输入张量
		infer_request.set_input_tensor(input_tensor);

		infer_request.infer(); // 执行推理

		auto output_tensor =
		    infer_request
		        .get_output_tensor(); // 获取输出结果
		auto output_data =
		    output_tensor
		        .data<float>(); // 假设输出是 float32 类型

		// 获取类别数
		size_t num_classes = output_tensor.get_shape()[1];

		// 输出所有类别的预测概率
		std::cout << "类别预测概率: " << std::endl;
		for(size_t i = 0; i < num_classes; ++i) {
			std::cout << "类别 " << i << ": "
			          << output_data[i] << std::endl;
		}

		// 输出最大概率的类别
		int predicted_class = std::distance(
		    output_data,
		    std::max_element(output_data,
		                     output_data + num_classes));
		std::cout << "预测类别: " << predicted_class
		          << std::endl; // 打印预测的类别

	} catch(const std::exception& ex) {
		std::cerr << "发生错误: " << ex.what() << std::endl;
		return -1;
	}

	return 0;
}
