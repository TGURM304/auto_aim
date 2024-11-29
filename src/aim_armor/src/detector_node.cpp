#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "armor_detector.hpp"

class ImageSubscriber: public rclcpp::Node {
public:
	ImageSubscriber(): Node("detector_node") {
		subscription_ = this->create_subscription<
		    sensor_msgs::msg::Image>(
		    "camera/stream", 10,
		    std::bind(&ImageSubscriber::image_callback,
		              this, std::placeholders::_1));
	}

private:
	void image_callback(
	    const sensor_msgs::msg::Image::SharedPtr msg) {
		cv_bridge::CvImagePtr cv_ptr;

		try {
			cv_ptr = cv_bridge::toCvCopy(
			    msg, sensor_msgs::image_encodings::BGR8);
		} catch(cv_bridge::Exception &e) {
			RCLCPP_ERROR(
			    this->get_logger(),
			    "Could not convert from '%s' to 'bgr8'.",
			    msg->encoding.c_str());
			return;
		}
		Armor_detector detector;
		detector.init();
	}

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
	    subscription_;
};


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageSubscriber>());
	rclcpp::shutdown();
	return 0;
}
