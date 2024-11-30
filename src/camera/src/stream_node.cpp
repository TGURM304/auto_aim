#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "mindvision.hpp"
#include "toml.hpp"

class StreamNode: public rclcpp::Node {
public:
	StreamNode(): Node("stream_node") {
		if(camera_version == "MV"){
			camera_.init(2);
		}	
		else if(camera_version == "HK"){
			;
		}

		publisher_ =
		    this->create_publisher<sensor_msgs::msg::Image>(
		        "camera/stream", 10);
		timer_ = this->create_wall_timer(
		    std::chrono::milliseconds(0),
		    std::bind(&StreamNode::publish, this));
	}

private:
	void publish() {
		if(camera_version == "MV"){
			frame = camera_.getFrame();
		}
		else if(camera_version == "HK"){
			;
		}
		else{
			frame = camera_.getFrame();
		}
		if(!frame.empty()) {
			auto msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8", frame).toImageMsg();
			publisher_->publish(*msg);
		}
	}

	toml::table config = toml::parse_file("./assets/config.toml");
	std::string camera_version = config["camera"]["version"].value_or("MV");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	MindVision camera_;
	cv::Mat frame;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StreamNode>());
	rclcpp::shutdown();
	return 0;
}
