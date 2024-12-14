#include "armor_detector.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>


ArmorDetector::ArmorDetector() {
	printf("ArmorDetector Start\n");
};

ArmorDetector::~ArmorDetector() {
	printf("ArmorDetector Shutdown\n");
};

int ArmorDetector::init() {
	try {
		toml::table config = toml::parse_file("assets/config.toml");

		// 相机内参 camera
		auto focal_toml = config["camera"]["focal"].as_array();
		auto center_toml = config["camera"]["center"].as_array();
		// 方便地读取焦距与光心信息
		// 焦距
		auto f = [&focal_toml](size_t i) {
			return focal_toml->get(i)->value_or(0.);
		};
		// 光心
		auto c = [&center_toml](size_t i) {
			return center_toml->get(i)->value_or(0.);
		};
		// clang-format off
		camera = {f(0),  0. , c(0),
		           0. , f(1), c(1),
		           0.,   0. ,  1. };
		// clang-format on

		// 畸变系数 dist
		auto dist_toml = config["camera"]["dist"].as_array();
		size_t dist_idx = 0;
		for(const auto& value: *dist_toml) {
			dist(0, dist_idx) = value.value_or(0.);
			dist_idx++;
		}

		// 初始化模型
		std::string model_path = config["aim_aromr"]["model_path"].value_or(
		    "assets/model/best-8.onnx");
		auto model = core.read_model(model_path);
		auto compiled_model = core.compile_model(model, "CPU");
		infer_request = compiled_model.create_infer_request();

		// 读取图像分类类别
		auto classes_array = config["aim_aromr"]["classes"].as_array();
		size_t classes_idx = 0;
		for(const auto& item: *classes_array) {
			if(item.is_string() && classes_idx < classes.size()) {
				classes[classes_idx] = item.as_string()->get();
				classes_idx++;
			}
		}
		return 0;
	} catch(const toml::parse_error& ex) {
		std::cerr << "Error parsing TOML file: " << ex.what() << std::endl;
		return -1;
	} catch(const std::exception& ex) {
		std::cerr << "Error initializing ArmorDetector: " << ex.what()
		          << std::endl;
		return -2;
	}
}
// TODO: 没有配置文件时的错误

std::string ArmorDetector::classify(const cv::Mat& image) {

	cv::resize(image, image, cv::Size(64, 64));
	// 归一化
	image.convertTo(image, CV_32F, 1.0 / 255.0);

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
cv::Point_<T> pcenter(const std::vector<cv::Point_<T>>& pnts) {
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

/**
 * @brief 转为齐次坐标
 * (x, y) -> (x, y, 1)
 */
cv::Vec3d h_(const cv::Point2d& p) {
	return cv::Vec3d(p.x, p.y, 1.);
}

ArmorCriterion ArmorDetector::rect_info(const std::vector<cv::Point2d>& kpnts,
                                        const cv::Matx33d& camera,
                                        const cv::Matx<double, 1, 5>& dist) {
	std::vector<cv::Point2d> ud_kpnts{};
	cv::undistortPoints(kpnts, ud_kpnts, camera, dist);

	cv::Matx33d C_inv;
	cv::invert(camera, C_inv);

	auto center = C_inv * h_(pcenter(ud_kpnts));

	double tmp = 0.;

	tmp = (C_inv * (h_(ud_kpnts[2]) - h_(ud_kpnts[0])))[0];
	if(tmp < 1e-6)
		return ArmorCriterion{.one_vote_no = true};
	double lambda1 = 2 * (center - C_inv * h_(ud_kpnts[0]))[0] / tmp;
	auto diag1 = lambda1 * h_(ud_kpnts[2]) - (2 - lambda1) * h_(ud_kpnts[0]);

	tmp = (C_inv * (h_(ud_kpnts[3]) - h_(ud_kpnts[1])))[0];
	if(tmp < 1e-6)
		return ArmorCriterion{.one_vote_no = true};
	double lambda2 = 2 * (center - C_inv * h_(ud_kpnts[1]))[0] / tmp;
	auto diag2 = lambda2 * h_(ud_kpnts[3]) - (2 - lambda2) * h_(ud_kpnts[1]);

	auto a = diag1 - diag2;
	auto b = diag1 + diag2;
	double la = cv::norm(a);
	double lb = cv::norm(b);

	auto oa = kpnts[2] - kpnts[0];
	auto ob = kpnts[3] - kpnts[1];
	double ola = cv::norm(oa);
	double olb = cv::norm(ob);
	double theta = std::acos(oa.dot(ob) / (ola * olb));

	using namespace std;
	using namespace std::numbers;
	return ArmorCriterion{
	    .one_vote_no = false,
	    .aspect_ratio = max(la / lb, lb / la),
	    .edge_angle = acos(a.dot(b) / (la * lb)),
	    .original_angle = min(theta, pi - theta),
	};
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
	                     0  , 1,       0      ,
	                     0  , 0,       1      };
	// clang-format on

	cv::Mat number_img;
	cv::warpPerspective(img, number_img, Cut * P, cv::Point(size, size));
	cv::Mat gray_number_img;
	cv::cvtColor(number_img, gray_number_img, cv::COLOR_BGR2GRAY);
	cv::threshold(gray_number_img, out, 0, 255,
	              cv::THRESH_OTSU | cv::THRESH_BINARY);
}
