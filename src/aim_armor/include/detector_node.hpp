#ifndef _AIMARMOR_H_DETECTOR_NODE_
#define _AIMARMOR_H_DETECTOR_NODE_


#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <interfaces/msg/target.hpp>
#include <interfaces/msg/aim_mode.hpp>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#elif defined(ROS_IRON)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "armors.hpp"
#include "lights.hpp"
#include "armor_detector.hpp"

using AimModeMsg = interfaces::msg::AimMode;
using ImageMsg = sensor_msgs::msg::Image;
using TargetMsg = interfaces::msg::Target;


struct AimMode {
	uint8_t mode;
	ArmorColor color;
};


/**
 * @brief 从下位机接受打击模式, 包括颜色与模式
 */
class GetModeNode: public rclcpp::Node {
public:
	/**
	 * @note `aim_mode` 的生命周期必须大于等于类本身
	 */
	GetModeNode(AimMode* aim_mode): Node("get_mode_node"), aim_mode_(aim_mode) {
		mode_sub_ = this->create_subscription<AimModeMsg>(
		    "/serial/mode", 10, [this](const AimModeMsg::SharedPtr msg) {
			    aim_mode_->mode = msg->mode;
			    aim_mode_->color =
			        msg->color == 'b' ? ArmorColor::BLUE : ArmorColor::RED;
		    });
	}

private:
	AimMode* aim_mode_;

	rclcpp::Subscription<AimModeMsg>::SharedPtr mode_sub_;
};


class DetectorNode: public rclcpp::Node {
public:
	DetectorNode(): Node("detector_sender_node") {
		aim_mode_.mode = 'a';
		aim_mode_.color = ArmorColor::RED;

		ad_.init();

		image_sub_ = this->create_subscription<ImageMsg>(
		    "/camera/stream", 10, [this](const ImageMsg::SharedPtr msg) {
			    try {
				    auto cv_ptr = cv_bridge::toCvCopy(
				        msg, sensor_msgs::image_encodings::BGR8);
				    process(cv_ptr->image);
			    } catch(cv_bridge::Exception& e) {
				    RCLCPP_ERROR(this->get_logger(),
				                 "Could not convert from '%s' to 'bgr8'.",
				                 msg->encoding.c_str());
			    }
		    });

		target_pub_ = this->create_publisher<TargetMsg>("/target/armor", 10);
		image_pub_ = this->create_publisher<ImageMsg>("/armors_image", 10);
	}

	/**
	 * @brief 获取控制识别模式的结构体指针
	 */
	AimMode* get_ref_aim_mode() {
		return &this->aim_mode_;
	}

private:
	void process(const cv::Mat& img);

	/**
	 * @brief 在图的左上角叠加字符串; 在中心画出红色瞄准点
	 */
	void draw_info_and_point(cv::Mat& img, const std::string& str);

	/**
	* @brief 计算方向角和距离
	* 
	* @param pos 在相机坐标系下目标的位置向量
	* @return 返回一个元组, 为 (pitch 俯仰, yaw 偏航, 距离)
	*/
	std::tuple<float, float, float> calc_pitch_yaw_dist(cv::Vec3f pos) {
		using namespace std;
		float n = cv::norm(pos);
		auto v = pos / n;
		return make_tuple(-asin(v[1]), -asin(v[0] / sqrt(1 - v[1] * v[1])), n);
	}

private:
	ArmorDetector ad_;

	AimMode aim_mode_;

	rclcpp::Subscription<ImageMsg>::SharedPtr image_sub_;
	rclcpp::Publisher<TargetMsg>::SharedPtr target_pub_;
	rclcpp::Publisher<ImageMsg>::SharedPtr image_pub_;
};


#endif