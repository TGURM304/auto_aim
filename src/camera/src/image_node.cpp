#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#elif defined(ROS_IRON)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif


using Image = sensor_msgs::msg::Image;
using namespace std::literals;

class StreamNode: public rclcpp::Node {
public:
	StreamNode(): Node("stream_node") {
		publisher_ = this->create_publisher<Image>("camera/stream", 10);
		timer_ = this->create_wall_timer(1s, [this]() {
			publisher_->publish(*img_msg_);
		});

		auto img = cv::imread(IMAGE_FILE_PATH);
		using namespace sensor_msgs::image_encodings;
		img_msg_ =
		    cv_bridge::CvImage(std_msgs::msg::Header(), BGR8, img).toImageMsg();
	}

private:
	const std::string IMAGE_FILE_PATH = "1.png";
	Image::SharedPtr img_msg_;

	rclcpp::Publisher<Image>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StreamNode>());
	rclcpp::shutdown();
	return 0;
}
