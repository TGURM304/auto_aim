#include <cassert>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <utility>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

#include "armor_detector.hpp"
#include "armors.hpp"
#include "lights.hpp"


#define RAD2DEG(rad) ((rad) / std::numbers::pi * 180.)


void save_image_with_time(const cv::Mat& img, const ArmorClasses category) {
	auto now = std::chrono::system_clock::now();
	auto timeval = std::chrono::time_point_cast<std::chrono::microseconds>(now)
	                   .time_since_epoch()
	                   .count();

	std::stringstream fname;
	fname << (size_t)category << "_" << timeval << ".jpg";

	// 保存图像
	if(cv::imwrite("tmp/outimg/" + fname.str(), img)) {
		std::cout << "图像已保存为 " << fname.str() << std::endl;
	} else {
		std::cout << "保存图像失败" << std::endl;
	}
}


ArmorDetector::ArmorDetector() {
	printf("ArmorDetector Start\n");
};

ArmorDetector::~ArmorDetector() {
	printf("ArmorDetector Shutdown\n");
};

int ArmorDetector::init() {
	try {
		toml::table config = toml::parse_file("assets/config.toml");

#define SET_ARG(var, name)                                                 \
	do {                                                                   \
		var##_cri[0].name =                                                \
		    config["aim_armor"][#var][#name].as_array()->get(0)->value_or( \
		        NAN);                                                      \
		var##_cri[1].name =                                                \
		    config["aim_armor"][#var][#name].as_array()->get(1)->value_or( \
		        NAN);                                                      \
		assert(var##_cri[0].name <= var##_cri[1].name);                    \
	} while(0)

		// 灯条检测所需参数
		// 从配置文件中导入
		SET_ARG(light, aspect_ratio);
		SET_ARG(light, area);
		SET_ARG(armor, aspect_ratio);
		SET_ARG(armor, edge_angle);
		SET_ARG(armor, original_angle);

		// 相机内参 camera
		auto focal_toml = config["camera"]["focal"].as_array();
		auto center_toml = config["camera"]["center"].as_array();
		// 方便地读取焦距与光心信息
		// 焦距
		auto f = [&focal_toml](size_t i) {
			auto val = focal_toml->get(i)->value_or(NAN);
			assert(!std::isnan(val));
			return val;
		};
		// 光心
		auto c = [&center_toml](size_t i) {
			auto val = center_toml->get(i)->value_or(NAN);
			assert(!std::isnan(val));
			return val;
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
		std::string model_path = config["aim_armor"]["model_path"].value_or(
		    "assets/model/best-8.onnx");
		auto model = core.read_model(model_path);
		auto compiled_model = core.compile_model(model, "CPU");
		infer_request = compiled_model.create_infer_request();

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

ArmorClasses ArmorDetector::classify(cv::Mat& image) {
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
	return (ArmorClasses)predicted_class;
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

std::vector<cv::Point2f> ArmorDetector::sort_points(const LinePoints2d& l1p,
                                                    const LinePoints2d& l2p) {
	LinePoints2d light_pnts[2] = {l1p, l2p};
	std::vector<cv::Point2f> sorted_pnts(4);

	// 求得四点的重心, 即对四点取平均
	cv::Point2d center = {0, 0};
	for(int i = 0; i < 4; i++)
		center += *((cv::Point2d*)light_pnts + i);
	center /= 4;

	for(int i = 0; i <= 1; i++) {
		auto light_pnt = light_pnts[i];
		auto pos = (light_pnt.first + light_pnt.second) / 2. - center;

		int a = pos.x > 0;
		int b = pcross(pos, light_pnt.first - center) < 0;
		sorted_pnts[(a << 1) + b] = light_pnt.first;
		sorted_pnts[(a << 1) + !b] = light_pnt.second;
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

ArmorCriterion ArmorDetector::rect_info(const std::vector<cv::Point2f>& kpnts,
                                        const cv::Matx33f& camera,
                                        const cv::Matx<float, 1, 5>& dist) {
	std::vector<cv::Point2f> ud_kpnts{};
	cv::undistortPoints(kpnts, ud_kpnts, camera, dist);

	cv::Matx33f C_inv;
	cv::invert(camera, C_inv);

	auto center = C_inv * h_(pcenter(ud_kpnts));

	float tmp = 0.;
	// 当计算出现问题时, 返回此值
	auto wrong_retval = ArmorCriterion{.one_vote_no = true,
	                                   .aspect_ratio = NAN,
	                                   .edge_angle = NAN,
	                                   .original_angle = NAN};

	tmp = (C_inv * (h_(ud_kpnts[2]) - h_(ud_kpnts[0])))[0];
	if(tmp < 1e-6)
		return wrong_retval;
	float lambda1 = 2 * (center - C_inv * h_(ud_kpnts[0]))[0] / tmp;
	auto diag1 = lambda1 * h_(ud_kpnts[2]) - (2 - lambda1) * h_(ud_kpnts[0]);

	tmp = (C_inv * (h_(ud_kpnts[3]) - h_(ud_kpnts[1])))[0];
	if(tmp < 1e-6)
		return wrong_retval;
	float lambda2 = 2 * (center - C_inv * h_(ud_kpnts[1]))[0] / tmp;
	auto diag2 = lambda2 * h_(ud_kpnts[3]) - (2 - lambda2) * h_(ud_kpnts[1]);

	auto a = diag1 - diag2;
	auto b = diag1 + diag2;
	float la = cv::norm(a);
	float lb = cv::norm(b);

	auto oa = kpnts[2] - kpnts[0];
	auto ob = kpnts[3] - kpnts[1];
	float ola = cv::norm(oa);
	float olb = cv::norm(ob);
	float theta = std::acos(oa.dot(ob) / (ola * olb));

	using namespace std;
	using namespace std::numbers;
	return ArmorCriterion{
	    .one_vote_no = false,
	    .aspect_ratio = max(la / lb, lb / la),
	    .edge_angle = RAD2DEG(acos(a.dot(b) / (la * lb))),
	    .original_angle = RAD2DEG(min(theta, (float)pi - theta)),
	};
}

void ArmorDetector::perspective(const cv::Mat& img, cv::Mat& out,
                                const std::vector<cv::Point2f>& kpnts,
                                int size) {
	float s = (float)size;
	std::vector<cv::Point2f> out_pnts = {{0., 0.}, {0., s}, {s, s}, {s, 0.}};

	auto P = cv::getPerspectiveTransform(kpnts, out_pnts);
	P.convertTo(P, CV_32F);

	float scale = 1.5;
	// clang-format off
	cv::Matx33f Cut = {scale, 0, (1-scale)*s/2,
	                     0  , 1,       0      ,
	                     0  , 0,       1      };
	// clang-format on

	// cv::Mat number_img;
	cv::warpPerspective(img, out, Cut * P, cv::Point(size, size));
	// cv::Mat gray_number_img;
	cv::cvtColor(out, out, cv::COLOR_BGR2GRAY);
	cv::threshold(out, out, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
	cv::cvtColor(out, out, cv::COLOR_GRAY2RGB);
}

std::optional<std::pair<cv::Vec3f, cv::Vec3f>> ArmorDetector::pnp_solver(
    const std::vector<cv::Point2f>& kpnts, ArmorSize armor_size,
    const cv::Matx33f& camera, const cv::Matx<float, 1, 5>& dist) {
	std::vector<cv::Point2f> img_pnts{};
	cv::undistortPoints(kpnts, img_pnts, camera, dist);

	std::vector<cv::Point3f> obj_pnts{};
	if(armor_size == ArmorSize::BIG) {
		// clang-format off
		obj_pnts = {{-BIG_ARMOR_WIDTH/2, -BIG_ARMOR_HEIGHT/2, 0},
		            {-BIG_ARMOR_WIDTH/2,  BIG_ARMOR_HEIGHT/2, 0},
		            { BIG_ARMOR_WIDTH/2,  BIG_ARMOR_HEIGHT/2, 0},
		            { BIG_ARMOR_WIDTH/2, -BIG_ARMOR_HEIGHT/2, 0}};
		// clang-format on
	} else if(armor_size == ArmorSize::SMALL) {
		// clang-format off
		obj_pnts = {{-SMALL_ARMOR_WIDTH/2, -SMALL_ARMOR_HEIGHT/2, 0},
		            {-SMALL_ARMOR_WIDTH/2,  SMALL_ARMOR_HEIGHT/2, 0},
		            { SMALL_ARMOR_WIDTH/2,  SMALL_ARMOR_HEIGHT/2, 0},
		            { SMALL_ARMOR_WIDTH/2, -SMALL_ARMOR_HEIGHT/2, 0}};
		// clang-format on
	}

	cv::Vec3f rvec, tvec;
	bool succ = cv::solvePnP(obj_pnts, img_pnts, camera, dist, rvec, tvec);
	if(!succ)
		return std::nullopt;

	// 再一次优化的提高非常有限
	//cv::solvePnPRefineLM(obj_pnts, img_pnts, camera, dist, rvec, tvec);

	cv::Matx33f R;
	Rodrigues(rvec, R);

	return std::make_pair(tvec, R * cv::Vec3f(0, 0, 1));
}

#define BETWEEN(x, l, r) ((l) <= (x) && (x) < (r))

#define CHECK_PARAM(name, var, field) \
	BETWEEN((var.field), (name##_cri[0].field), (name##_cri[1].field))

bool ArmorDetector::light_check(const LightCriterion& lc) {
	return !lc.one_vote_no && CHECK_PARAM(light, lc, aspect_ratio)
	    && CHECK_PARAM(light, lc, area);
}

bool ArmorDetector::armor_check(const ArmorCriterion& ac) {
	return !ac.one_vote_no && CHECK_PARAM(armor, ac, aspect_ratio)
	    && CHECK_PARAM(armor, ac, edge_angle)
	    && CHECK_PARAM(armor, ac, original_angle);
}

void ArmorDetector::preprocess(cv::Mat& out, const cv::Mat& in) {
	constexpr int thresh = 160;
	constexpr int kernel_size = 5;

	cv::Mat gray_img;
	cv::cvtColor(in, gray_img, cv::COLOR_BGR2GRAY);
	cv::Mat bin_img;
	cv::threshold(gray_img, bin_img, thresh, 255, cv::THRESH_BINARY);
	cv::morphologyEx(bin_img, out, cv::MORPH_OPEN,
	                 cv::Mat::ones(kernel_size, kernel_size, CV_8U));
}

size_t ArmorDetector::match_armors(std::vector<Armor>& armors,
                                   const cv::Mat& img, ArmorColor color) {
	using namespace std::placeholders;

	cv::Mat bin_img;
	preprocess(bin_img, img);

	std::vector<Contour> cnts;
	std::vector<cv::Vec4i> _;
	cv::findContours(bin_img, cnts, _, cv::RETR_EXTERNAL,
	                 cv::CHAIN_APPROX_SIMPLE);

	std::vector<Light> lights;
	auto check = std::bind(&ArmorDetector::light_check, this, _1);
	for(auto& cnt: cnts) {
		auto tmp = Light::try_from_contour(cnt, img, check);
		if(!tmp.has_value())
			continue;
		auto light = tmp.value();
		if(light.color != color)
			continue;
		lights.push_back(light);
	}

	size_t cnt = 0;
	for(size_t i = 0; i < lights.size(); i++) {
		for(size_t j = i + 1; j < lights.size(); j++) {
			auto l1 = lights[i];
			auto l2 = lights[j];

			auto kpnts_fig = sort_points(l1.full_points(), l2.full_points());
			auto kpnts_pnp = sort_points(l1.points(), l2.points());
			auto armor_cri = rect_info(kpnts_fig, camera, dist);

			if(!armor_check(armor_cri))
				continue;

			cv::Mat fig;
			perspective(img, fig, kpnts_fig, 64);
			// cv::Mat fig_save = fig.clone();
			auto classes = classify(fig);

			// // 调试使用
			// // DEBUG
			// if(!fig.empty()) {
			// 	save_image_with_time(fig_save, classes);
			// }

			ArmorSize size;
			switch(classes) {
			case NUM1:
			case BASE:
				size = ArmorSize::BIG;
				break;
			case NUM2:
			case NUM3:
			case NUM4:
			case QSZ:
			case SB:
			case NONE:
				size = ArmorSize::SMALL;
				break;
			default:
				continue;
			};
			// TODO: 类别与装甲板长宽比综合判断
			auto tmp = pnp_solver(kpnts_pnp, size, camera, dist);
			if(!tmp.has_value())
				continue;

			armors.emplace_back(classes, tmp.value().first, tmp.value().second);
			cnt++;
		}
	}

	return cnt;
}
