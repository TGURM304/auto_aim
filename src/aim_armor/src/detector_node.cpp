#include <algorithm>

#include "detector_node.hpp"


void DetectorNode::process(const cv::Mat &img) {
	using namespace std;
	using namespace cv;
	using namespace cv_bridge;
	using namespace std_msgs::msg;
	using namespace sensor_msgs::image_encodings;

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
		return;
	}
	float pitch, yaw, dist;
	tie(pitch, yaw, dist) = calc_pitch_yaw_dist(armor->pos);

	// 在图上标上信息, 在中心标上瞄准点
	auto info = "mode:" + to_string(aim_mode_.mode) + "|dist:" + to_string(dist)
	    + "|pitch:" + to_string(pitch) + "|yaw:" + to_string(yaw);
	draw_info_and_point(out_img, info);

	// 发布击打目标
	TargetMsg t;
	t.aim_mode = aim_mode_.mode;
	t.pitch_angle = RAD2DEG(pitch);
	t.yaw_angle = RAD2DEG(yaw);
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