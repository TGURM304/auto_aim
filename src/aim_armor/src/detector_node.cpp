#include <sys/types.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <interfaces/msg/target.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <interfaces/msg/aim_mode.hpp>
#include <string>
#include <utility>
#include <vector>
#include "armors.hpp"

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#elif defined(ROS_IRON)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "lights.hpp"
#include "armor_detector.hpp"

using AimModemsg = interfaces::msg::AimMode;
using ImageMsg = sensor_msgs::msg::Image;
using TargetMsg = interfaces::msg::Target;


class Detector {
public:
	// 从下位机接收的击打模式
	struct RunMode {
		// TODO: 默认应该是什么值
		ArmorColor color = RED;
		u_int8_t mode;
	} runmode;

public:
	Detector() {
		ad.init();
	};
	~Detector() {};

	cv::Mat frame;
	cv::Mat output_img;
	std::vector<Armor> armors;

	std::pair<std::vector<Armor>, cv::Mat> get_armors() {
		armors.clear();
		if(!frame.empty()) {
			ad.match_armors(armors, frame, runmode.color, output_img);
		}
		return std::make_pair(armors, output_img);
	}

private:
	ArmorDetector ad;
};

class Tracker {
public:
	// 状态
	enum State { SEARCHING, TRACKING };

	// 打击目标结构体（发送的数据）
	struct Target {
		uint8_t aim_mode;
		float pitch_angle;
		float yaw_angle;
		float distance;
	} target;

public:
	Tracker() {
		target.aim_mode = 0;
		target.pitch_angle = .0;
		target.yaw_angle = .0;
	};

	~Tracker() {};

	/**
	* @brief 计算方向角和距离
	* 
	* @param pos 在相机坐标系下目标的位置向量
	* @return 返回一个元组, 为 (pitch 俯仰, yaw 偏航, 距离)
	*/
	std::tuple<float, float, float> calcPitchYawDist(cv::Vec3f pos) {
		using namespace std;
		float n = cv::norm(pos);
		auto v = pos / n;
		return make_tuple(-asin(v[1]), -asin(v[0] / sqrt(1 - v[1] * v[1])), n);
	}

	Target tracking(std::vector<Armor> armors) {
		if(armors.size() == 0) {
			target.aim_mode = '0';
			return target;
		} else {
			target.aim_mode = 'a';
			std::tie(target.pitch_angle, target.yaw_angle, target.distance) =
			    calcPitchYawDist(armors[0].pos);
			target.pitch_angle = target.pitch_angle / std::numbers::pi * 180,
			target.yaw_angle = target.yaw_angle / std::numbers::pi * 180;
			armors.clear();
			return target;
		}
	}

private:
	// 当前跟踪状态
	State currentState = SEARCHING;
	// 跟踪的装甲板
	struct TrackedArmor {
		ArmorClasses classes;
		float distance;
	} previousArmor, currentArmor;

	bool isSameBoard(TrackedArmor &armor) {
		// TODO: 更改目标跟踪的算法,目前是非常弱智的小学生写法
		float distanceThreshold = 99999; // 距离阈值
		// 比较装甲板的编号和距离
		if(armor.classes == previousArmor.classes) {
			if(std::abs(armor.distance - previousArmor.distance)
			   < distanceThreshold) {
				return true;
			}
			return true;
		}
		return false;
	}
};

class Target_Sender_Node: public rclcpp::Node {
public:
	Target_Sender_Node(Detector &detector, Tracker &trakcer):
	Node("detector_sender_node"), detector_(detector), tracker_(trakcer) {
		subscription_ = this->create_subscription<ImageMsg>(
		    "camera/stream", 10,
		    std::bind(&Target_Sender_Node::get_target, this,
		              std::placeholders::_1));
		target_publisher_ =
		    this->create_publisher<TargetMsg>("/target/armor", 10);
		image_publisher_ =
		    this->create_publisher<ImageMsg>("/armors_image", 10);
	}

private:
	void get_target(const ImageMsg::SharedPtr msg) {
		try {
			cv_ptr =
			    cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			detector_.frame = cv_ptr->image;
		} catch(cv_bridge::Exception &e) {
			RCLCPP_ERROR(this->get_logger(),
			             "Could not convert from '%s' to 'bgr8'.",
			             msg->encoding.c_str());
			return;
		}

		auto detector_results = detector_.get_armors();

		auto tracker_result = tracker_.tracking(detector_results.first);

		target.aim_mode = tracker_result.aim_mode;
		target.pitch_angle = tracker_result.pitch_angle;
		target.yaw_angle = tracker_result.yaw_angle;
		target.distance = tracker_result.distance;

		target_publisher_->publish(target);


		int width = detector_results.second.cols;
		int height = detector_results.second.rows;
		cv::Point center(width / 2, height / 2);
		cv::circle(detector_results.second, center, 3, cv::Scalar(0, 0, 255),
		           -1);


		if(detector_results.second.cols < 800) {
			cv::putText(
			    detector_results.second,
			    "mode:" + std::to_string(tracker_result.aim_mode)
			        + " | dis:" + std::to_string(tracker_result.distance)
			        + " | pitch:" + std::to_string(tracker_result.pitch_angle)
			        + " | yaw:" + std::to_string(tracker_result.yaw_angle),
			    cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
			    cv::Scalar(255, 255, 255), 1);
		} else {
			cv::putText(
			    detector_results.second,
			    "mode:" + std::to_string(tracker_result.aim_mode)
			        + " | dis:" + std::to_string(tracker_result.distance)
			        + " | pitch:" + std::to_string(tracker_result.pitch_angle)
			        + " | yaw:" + std::to_string(tracker_result.yaw_angle),
			    cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, 1,
			    cv::Scalar(255, 255, 255), 2);
		}


		ImageMsg::SharedPtr image_msg =
		    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",
		                       detector_results.second)
		        .toImageMsg();
		image_publisher_->publish(*image_msg);
	}

	Detector detector_;
	Tracker tracker_;
	TargetMsg target;
	cv_bridge::CvImagePtr cv_ptr;
	rclcpp::Subscription<ImageMsg>::SharedPtr subscription_;
	rclcpp::Publisher<TargetMsg>::SharedPtr target_publisher_;
	rclcpp::Publisher<ImageMsg>::SharedPtr image_publisher_;
};


class Get_Mode_Node: public rclcpp::Node {
public:
	Get_Mode_Node(Detector &detector):
	Node("serial_sender_node"), detector_(detector) {
		// 订阅目标消息
		subscription = this->create_subscription<AimModemsg>(
		    "/serial/mode", 10,
		    std::bind(&Get_Mode_Node::setmode, this, std::placeholders::_1));
	}

private:
	void setmode(const AimModemsg::SharedPtr msg) {
		detector_.runmode.mode = msg->mode;
		if(msg->color == 'r') {
			detector_.runmode.color = RED;
		} else if(msg->color == 'b') {
			detector_.runmode.color = BLUE;
		} else {
			detector_.runmode.mode = RED;
		}
	}

	Detector &detector_;
	rclcpp::Subscription<AimModemsg>::SharedPtr subscription;
	rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char **argv) {
	Detector detector;
	Tracker tracker;
	rclcpp::init(argc, argv);

	auto detector_node =
	    std::make_shared<Target_Sender_Node>(detector, tracker);
	auto get_mode_node = std::make_shared<Get_Mode_Node>(detector);

	// 使用多线程执行器来管理节点
	rclcpp::executors::MultiThreadedExecutor executor;
	// 添加节点到执行器
	executor.add_node(detector_node);
	executor.add_node(get_mode_node);
	// 启动执行器
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
