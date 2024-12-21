#include <opencv2/core/matx.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <interfaces/msg/armor.hpp>
#include <interfaces/msg/armors.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "armor_detector.hpp"
#include "armors.hpp"
#include "lights.hpp"


using ImageMsg = sensor_msgs::msg::Image;
using ArmorsMsg = interfaces::msg::Armors;
using ArmorMsg = interfaces::msg::Armor;


class Detector {
public:
	Detector(){};
	~Detector(){};

	cv::Mat frame;
	std::vector<ArmorMsg> armors;

	std::vector<ArmorMsg> get_armors() {
		// test
		// armors.clear();
		// armor_msg.number = "1";
		// armor_msg.distance_to_image_center = .0;
		// armors.push_back(armor_msg);
		
		return armors;
	}

private:
	ArmorMsg armor_msg;
};


class Detector_Sender_Node: public rclcpp::Node {
public:
	Detector_Sender_Node(Detector &detector):
	Node("detector_sender_node"), detector_(detector) {
		subscription_ = this->create_subscription<ImageMsg>(
		    "camera/stream", 10,
		    std::bind(&Detector_Sender_Node::detector, this,
		              std::placeholders::_1));
		armors_publisher_ =
		    this->create_publisher<ArmorsMsg>("/detector/armors", 10);
	}

private:
	void detector(const ImageMsg::SharedPtr msg) {
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
		armors_msg.armors = detector_.get_armors();
		armors_msg.header.frame_id = "1";
		armors_msg.header.stamp = this->get_clock()->now();
		armors_publisher_->publish(armors_msg);
	}

	Detector detector_;
	ArmorsMsg armors_msg;
	cv_bridge::CvImagePtr cv_ptr;
	rclcpp::Subscription<ImageMsg>::SharedPtr subscription_;
	rclcpp::Publisher<ArmorsMsg>::SharedPtr armors_publisher_;
};


int main(int argc, char **argv) {
	Detector detector;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Detector_Sender_Node>(detector));
	rclcpp::shutdown();
	return 0;
}
