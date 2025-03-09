#include <chrono>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <variant>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#elif defined(ROS_IRON)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "mindvision.hpp"
#include "hikvision.hpp"
#include "toml.hpp"

using namespace std;


class StreamNode: public rclcpp::Node {
public:
	StreamNode(): Node("stream_node") {
		auto type = this->declare_parameter("camera.version", "MV");
		if(type == "MV")
			type_ = MV;
		else if(type == "HK")
			type_ = HK;

		if(type_ == MV) {
			camera_ = MindVision{};
			get<MindVision>(camera_).init(2);
		} else if(type_ == HK) {
			camera_ = HikVision{};
			int err;
			while((err = get<HikVision>(camera_).init()) != MV_OK)
				RCLCPP_ERROR(this->get_logger(), "Error code: 0x%x", err);
		} else {
			RCLCPP_ERROR(
			    this->get_logger(),
			    "配置文件错误, 无法识别的相机类型: %s, 尝试重新获取配置\n",
			    type.c_str());
		}

		publisher_ =
		    this->create_publisher<sensor_msgs::msg::Image>("camera/stream", 1);
		timer_ = this->create_wall_timer(chrono::milliseconds(1),
		                                 bind(&StreamNode::publish, this));
	}

private:
	void publish() {
		if(type_ == MV) {
			frame_ = get<MindVision>(camera_).getFrame();
		} else if(type_ == HK) {
			auto result = get<HikVision>(camera_).getFrame();
			if(result.second == MV_OK) {
				frame_ = result.first;
			} else {
				frame_ = result.first;
				RCLCPP_ERROR(this->get_logger(), "Error code: 0x%x",
				             result.second);
			}
		} else {
			frame_ = get<MindVision>(camera_).getFrame();
		}

		if(!frame_.empty()) {
			auto msg =
			    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_)
			        .toImageMsg();
			publisher_->publish(*msg);
		} else {
			RCLCPP_INFO(this->get_logger(), "Empty frame\n");
		}
	}

private:
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

private:
	toml::table config;
	enum { MV, HK } type_;
	std::variant<MindVision, HikVision> camera_;
	cv::Mat frame_;
};


int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StreamNode>());
	rclcpp::shutdown();
	return 0;
}
