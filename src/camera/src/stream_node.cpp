#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "mindvision.cpp"

class stream_node: public rclcpp::Node {
public:
	stream_node(): Node("stream_node") {
		publisher_ =
		    this->create_publisher<sensor_msgs::msg::Image>(
		        "camera/stream", 10);
		timer_ = this->create_wall_timer(
		    std::chrono::milliseconds(1),
		    std::bind(&stream_node::publish, this));
	}

private:
	void publish() {
		int hCamera = camera.Init(2);
		cv::Mat frame = camera.getMindvision(hCamera);
		if(!frame.empty()) {
			auto msg =
			    cv_bridge::CvImage(std_msgs::msg::Header(),
			                       "bgr8", frame)
			        .toImageMsg();
			publisher_->publish(*msg);
		}
	}

	MindVision camera;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
	    publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};




int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<stream_node>());
	rclcpp::shutdown();
	return 0;
}
