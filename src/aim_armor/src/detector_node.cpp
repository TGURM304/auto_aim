#include <algorithm>
#include <cmath>
#include <optional>

#include "detector_node.hpp"


// // FIXME: 配置导入
// #define G         9.8
// #define V0        NAN
// #define MU        NAN
// #define THETA_MAX NAN
// #define THETA_MIN NAN


// std::optional<float> calc_track(float dist, float theta) {
// 	using namespace std;

// 	float epsilon = 1e-3;
// 	float rx = dist * cos(theta);
// 	float ry = dist * sin(theta);

// 	if(rx <= 0.)
// 		return nullopt;
// 	if(atan2(ry, rx) > THETA_MAX)
// 		return nullopt;
// 	if(ry > -G / (2 * V0 * V0) * rx * rx + V0 * V0 / (2 * G))
// 		return nullopt;
// 	if(V0 > G / MU && rx > V0 / sqrt(MU * MU - (G / V0) * (G / V0)))
// 		return nullopt;

// 	auto f = [rx, ry](float phi) {
// 		return ry
// 		    - G
// 		    * (MU * rx / cos(phi) - V0 * log(V0 / (V0 - MU * rx / cos(phi))))
// 		    / (V0 * MU * MU)
// 		    - rx * tan(phi);
// 	};
// 	auto f_p = [rx](float phi) {
// 		return rx / (cos(phi) * cos(phi))
// 		    * (G * rx * sin(phi) / (V0 * V0 * cos(phi) - MU * V0 * rx) - 1.);
// 	};

// 	float theta_st_min = atan2(V0 * V0, G * rx)
// 	    - atan2(rx * MU,
// 	            sqrt(V0 * V0 + rx * rx * ((G / V0) * (G / V0) - MU * MU)));
// 	theta = theta_st_min < THETA_MIN ? THETA_MAX : THETA_MIN;
// 	float f_theta = f(theta);
// 	float f_theta_m = f(theta_st_min);

// 	theta =
// 	    (theta * f_theta_m - theta_st_min * f_theta) / (f_theta_m - f_theta);
// 	for(int _ = 0; _ < 5; _++) {
// 		theta -= f(theta) / f_p(theta);
// 	}

// 	if(!(THETA_MIN <= theta || theta <= THETA_MAX))
// 		return nullopt;
// 	if(abs(f(theta)) > epsilon)
// 		return nullopt;

// 	return make_optional(theta);
// }

void DetectorNode::process(const cv::Mat &img) {
	using namespace std;
	using namespace cv;
	using namespace cv_bridge;
	using namespace std_msgs::msg;
	using namespace sensor_msgs::image_encodings;

	TargetMsg t{};

	float yaw_current = aim_mode_.yaw;
	float pitch_current = aim_mode_.pitch;

	// 获取全部装甲板, 在图上标记每个板
	vector<Armor> armors;
	Mat out_img;
	ad_.match_armors(armors, img, aim_mode_.color, &out_img);

	// 发布结果图像
	auto image_msg = CvImage(Header(), BGR8, out_img).toImageMsg();
	image_pub_->publish(*image_msg);

	// 找出距画面中心点最近的装甲板
	auto d = [](Vec3d v) {
		return sqrt(v[0] * v[0] + v[1] * v[1]) / v[2];
	};
	auto armor = min_element(armors.begin(), armors.end(), [d](auto x, auto y) {
		return d(x.pos) > d(y.pos);
	});
	if(armor == armors.end()) {
		target_pub_->publish(t);
		return;
	}
	float pitch, yaw, dist;
	tie(pitch, yaw, dist) = calc_pitch_yaw_dist(armor->pos);

	// auto val = calc_track(dist, pitch);
	// if(!val.has_value()) {
	// 	target_pub_->publish(t);
	// 	return;
	// }
	// pitch = val.value();

	// 在图上标上信息, 在中心标上瞄准点
	auto info = "mode:" + to_string(aim_mode_.mode) + "|dist:" + to_string(dist)
	    + "|pitch:" + to_string(pitch) + "|yaw:" + to_string(yaw);
	draw_info_and_point(out_img, info);

	// 发布击打目标
	t.aim_mode = aim_mode_.mode;
	t.pitch_angle = RAD2DEG(pitch) + pitch_current;
	t.yaw_angle = RAD2DEG(yaw) + yaw_current;
	t.distance = dist;
	target_pub_->publish(t);
}

void DetectorNode::draw_info_and_point(cv::Mat &img, const std::string &str) {
	cv::circle(img, cv::Point(img.cols / 2, img.rows / 2), 3,
	           cv::Scalar(0, 0, 255), -1);

	float scale;
	int thickness;

	if(img.cols < 800) {
		scale = 0.5;
		thickness = 1;
	} else {
		scale = 1.;
		thickness = 2;
	}

	cv::putText(img, str, cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, scale,
	            cv::Scalar(255, 255, 255), thickness);
}


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto detector_node = std::make_shared<DetectorNode>();
	auto get_mode_node =
	    std::make_shared<GetModeNode>(detector_node->get_ref_aim_mode());

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(get_mode_node);
	executor.add_node(detector_node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
