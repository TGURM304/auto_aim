#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "mindvision.hpp"


class stream_node: public rclcpp::Node {
public:
	stream_node(): Node("stream_node") {
		publisher_ =
		    this->create_publisher<sensor_msgs::msg::Image>(
		        "camera/stream", 10);
		timer_ = this->create_wall_timer(
		    std::chrono::milliseconds(0),
		    std::bind(&stream_node::publish, this));
	}

private:
	void publish() {
		int camera = camera_.init(2);
		cv::Mat frame = camera_.getFrame(camera);
		if(!frame.empty()) {
			auto msg =
			    cv_bridge::CvImage(std_msgs::msg::Header(),
			                       "bgr8", frame)
			        .toImageMsg();
			publisher_->publish(*msg);
		}
	}

	MindVision camera_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
	    publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};
// TODO: 声明与实现分离


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<stream_node>());
	rclcpp::shutdown();
	return 0;
}
