#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

using Image = sensor_msgs::msg::Image;
using namespace std::literals;


class gray_node: public rclcpp::Node {
public:
	gray_node(): Node("gray_node") {
		publisher_ = this->create_publisher<Image>(
		    "camera/gray_stream", 10);
		timer_ = this->create_wall_timer(
		    1ms,
		    std::bind(&gray_node::publish_image, this));
		cap_.open(0);
	}

private:
	void publish_image() {
		cv::Mat frame;
		cap_ >> frame;
		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		if(!frame.empty()) {
			auto msg =
			    cv_bridge::CvImage(std_msgs::msg::Header(),
			                       "mono8", frame)
			        .toImageMsg();
			publisher_->publish(*msg);
		}
	}

	cv::VideoCapture cap_;
	rclcpp::Publisher<Image>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};
// TODO: 声明与实现分离


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<gray_node>());
	rclcpp::shutdown();
	return 0;
}
