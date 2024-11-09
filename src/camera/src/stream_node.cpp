#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "mindvision.hpp"

class stream_node: public rclcpp::Node {
public:
    stream_node(): Node("stream_node") {
        
          
        if (hCamera < 0) {
            RCLCPP_ERROR(this->get_logger(), "Camera initialization failed");
        }

        publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>("camera/stream", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&stream_node::publish, this));
    }
	~stream_node(){
		camera_.~MindVision();
	}
private:
    void publish() {
        cv::Mat frame = camera_.getFrame(hCamera);
        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Empty frame received");
        }
    }

    MindVision camera_;
	int hCamera = camera_.init(2);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stream_node>());
    rclcpp::shutdown();
    return 0;
}
