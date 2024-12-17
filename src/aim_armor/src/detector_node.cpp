#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <interfaces/msg/armor.hpp>
#include <interfaces/msg/armors.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>


#include "armor_detector.hpp"
#include "lights.hpp"


using ImageMsg = sensor_msgs::msg::Image;
using ArmorsMsg = interfaces::msg::Armors;
using ArmorMsg = interfaces::msg::Armor;


class DetectorNode: public rclcpp::Node {
public:
	DetectorNode(): Node("detector_node") {
		subscription_ = this->create_subscription<ImageMsg>(
		    "camera/stream", 10, std::bind(&DetectorNode::detector, this, std::placeholders::_1));

		armors_publisher_ =
		    this->create_publisher<ArmorsMsg>("detector/armors", 10);
		armorDetector.init();
	}
	

private:
	void detector(const ImageMsg::SharedPtr msg) {
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			frame = cv_ptr->image;
		} catch(cv_bridge::Exception &e) {
			RCLCPP_ERROR(this->get_logger(),
			             "Could not convert from '%s' to 'bgr8'.",
			             msg->encoding.c_str());
			return;
		}

		light.from_contour(contour, frame);
		std::cout << contour << std::endl;

		std::vector<ArmorMsg> armors;
		for(int i = 0; i < 3; ++i) {
			ArmorMsg armor;
			armor.number = "1";
			armor.type = "BIG";
			armor.distance_to_image_center = 100.0f;

			armor.pose.position.x = i * 100.0;
			armor.pose.position.y = i * 50.0;
			armor.pose.position.z = 0.0;
			armor.pose.orientation.x = 0.0;
			armor.pose.orientation.y = 0.0;
			armor.pose.orientation.z = 0.0;
			armor.pose.orientation.w = 1.0;

			armor.kpts.push_back(geometry_msgs::msg::Point());
			armor.kpts.back().x = i * 10.0;
			armor.kpts.back().y = i * 5.0;
			armor.kpts.back().z = 0.0;

			armors.push_back(armor);
		}

		ArmorsMsg armors_msg;
		armors_msg.header.stamp = this->get_clock()->now();
		armors_msg.header.frame_id = "armors_msg";
		armors_msg.armors = armors;

		armors_publisher_->publish(armors_msg);
	}

	// 初始化armordetector
	ArmorDetector armorDetector;
	//初始化lights
	Light light;
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat frame;
	std::vector<cv::Point> contour;
	rclcpp::Subscription<ImageMsg>::SharedPtr subscription_;
	rclcpp::Publisher<ArmorsMsg>::SharedPtr armors_publisher_;
};


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DetectorNode>());
	rclcpp::shutdown();
	return 0;
}
