#include "armor_detector.hpp"

#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>


ArmorDetector::ArmorDetector() {
	printf("ArmorDetector Start\n");
};

ArmorDetector::~ArmorDetector() {
	printf("ArmorDetector Shutdown\n");
};

int ArmorDetector::init() {
	try {
		// 读取配置
		std::string model_path = config["aim_aromr"]["model_path"].value_or(
		    "./assets/model/best-8.onnx");
		auto classes_array = config["aim_aromr"]["classes"].as_array();

		// 初始化模型
		auto model = core.read_model(model_path);
		auto compiled_model = core.compile_model(model, "CPU");
		infer_request = compiled_model.create_infer_request();

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
		std::cerr << "Error parsing TOML file: " << ex.what() << std::endl;
		return -1;
	} catch(const std::exception& ex) {
		std::cerr << "Error initializing ArmorDetector: " << ex.what()
		          << std::endl;
		return -1;
	}
}

std::string ArmorDetector::classify(const cv::Mat& image) {

	cv::resize(image, image, cv::Size(64, 64));
	image.convertTo(image, CV_32F, 1.0 / 255.0); // 归一化
	// 转换图像格式为 Tensor
	ov::Shape input_shape = {1, 64, 64, 3}; // NHWC
	ov::Tensor input_tensor(ov::element::f32, input_shape, image.data);
	infer_request.set_input_tensor(input_tensor);

	// 推理并获取结果
	infer_request.infer();
	auto output_tensor = infer_request.get_output_tensor();
	auto output_data = output_tensor.data<float>();
	int predicted_class = std::distance(
	    output_data,
	    std::max_element(output_data, output_data + output_tensor.get_size()));
	return classes[predicted_class];
}

/**
 * @brief 计算 2 维向量的外积, 返回一个标量
 */
template<typename T>
T pcross(cv::Point_<T> x, cv::Point_<T> y) {
	return x.x * y.y - x.y * y.x;
}

/**
 * @brief 计算 2 维向量的外积, 返回一个标量
 */
template<typename T>
T vcross(cv::Vec<T, 2> x, cv::Vec<T, 2> y) {
	return x[0] * y[1] - x[1] * y[0];
}

/**
 * @brief 计算 4 点的中点
 *
 * 此处所指的中点, 为两组对角线的交点
 *
 * @warning `pnts` 数组的长度必须等于 4
 */
template<typename T>
cv::Point_<T> pcenter(const cv::Point_<T> pnts[]) {
	auto p0 = pnts[0];
	T mu = pcross(pnts[3] - p0, pnts[1] - p0)
	    / pcross(pnts[2] - p0, pnts[1] - pnts[3]);
	return p0 + mu * pnts[2];
}

std::vector<cv::Point2d> ArmorDetector::sort_points(const Light& l1,
                                                    const Light& l2) {
	cv::Point2d light_pnts[2][2] = {{l1.pos + l1.offset, l1.pos - l1.offset},
	                                {l2.pos + l2.offset, l2.pos - l2.offset}};
	std::vector<cv::Point2d> sorted_pnts(4);

	// 求得四点的重心, 即对四点取平均
	cv::Point2d center = {0, 0};
	for(int i = 0; i < 4; i++)
		center += *((cv::Point2d*)light_pnts + i);
	center /= 4;

	for(int i = 0; i <= 1; i++) {
		auto light_pnt = light_pnts[i];
		auto pos = (light_pnt[0] + light_pnt[1]) / 2. - center;

		int a = pos.x > 0;
		int b = pcross(pos, light_pnt[0] - center) < 0;
		sorted_pnts[(a << 1) + b] = light_pnt[0];
		sorted_pnts[(a << 1) + !b] = light_pnt[1];
	}

	return sorted_pnts;
}

void ArmorDetector::perspective(const cv::Mat& img, cv::Mat& out,
                                const std::vector<cv::Point2d>& kpnts,
                                int size) {
	double s = (double)size;
	std::vector<cv::Point2d> out_pnts = {{0., 0.}, {0., s}, {s, s}, {s, 0.}};
	auto P = cv::getPerspectiveTransform(kpnts, out_pnts);

	double scale = 1.8;
	// clang-format off
	cv::Matx33d Cut = {scale, 0, (1-scale)*s/2,
	                       0, 1,             0,
	                       0, 0,             1};
	// clang-format on

	Mat number_img;
	cv::warpPerspective(img, number_img, Cut * P, cv::Point(size, size));
	Mat gray_number_img;
	cv::cvtColor(number_img, gray_number_img, cv::COLOR_BGR2GRAY);
	cv::threshold(gray_number_img, out, 0, 255,
	              cv::THRESH_OTSU | cv::THRESH_BINARY);
}
