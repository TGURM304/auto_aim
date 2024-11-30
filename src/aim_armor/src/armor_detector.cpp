#include "armor_detector.hpp"


ArmorDetector::ArmorDetector() {
	printf("ArmorDetector Start\n");
};

ArmorDetector::~ArmorDetector() {
	printf("ArmorDetector Shutdown\n");
};

int ArmorDetector::init() {
	try {
		// 读取配置
		std::string model_path =
		    config["aim_aromr"]["model_path"].value_or(
		        "./assets/model/best-8.onnx");
		auto classes_array =
		    config["aim_aromr"]["classes"].as_array();

		// 初始化模型
		auto model = core.read_model(model_path);
		auto compiled_model =
		    core.compile_model(model, "CPU");
		infer_request =
		    compiled_model.create_infer_request();

		// 读取图像分类类别
		size_t index = 0;
		for(const auto& item: *classes_array) {
			if(item.is_string() && index < classes.size()) {
				classes[index] = item.as_string()->get();
				++index;
			}
		}
		return 1;
	} catch(const toml::parse_error& ex) {
		std::cerr << "Error parsing TOML file: "
		          << ex.what() << std::endl;
		return -1;
	} catch(const std::exception& ex) {
		std::cerr << "Error initializing ArmorDetector: "
		          << ex.what() << std::endl;
		return -1;
	}
}

std::string ArmorDetector::classify(cv::Mat image) {

	cv::resize(image, image, cv::Size(64, 64));
	image.convertTo(image, CV_32F, 1.0 / 255.0); // 归一化
	// 转换图像格式为 Tensor
	ov::Shape input_shape = {1, 64, 64, 3}; // NHWC
	ov::Tensor input_tensor(ov::element::f32, input_shape,
	                        image.data);
	infer_request.set_input_tensor(input_tensor);

	// 推理并获取结果
	infer_request.infer();
	auto output_tensor = infer_request.get_output_tensor();
	auto output_data = output_tensor.data<float>();
	int predicted_class = std::distance(
	    output_data,
	    std::max_element(
	        output_data,
	        output_data + output_tensor.get_size()));
	return classes[predicted_class];
}
