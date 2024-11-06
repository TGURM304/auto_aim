#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

using Image = sensor_msgs::msg::Image;
using namespace std::literals;


class StreamNode: public rclcpp::Node {
public:
	StreamNode(): Node("stream_node") {
		publisher_ = this->create_publisher<Image>(
		    "camera/stream", 10);
		timer_ = this->create_wall_timer(
		    25ms, std::bind(&StreamNode::publish, this));

		// 打开视频文件
		// TODO: 通过宏传入视频文件地址
		cap_.open("./1.mp4");
		if(!cap_.isOpened()) {
			RCLCPP_ERROR(this->get_logger(),
			             "Failed to open video file");
			return;
		}

		// 获取视频帧总数
		total_frames_ = static_cast<int>(
		    cap_.get(cv::CAP_PROP_FRAME_COUNT));
		frame_counter_ = 0;
	}

private:
	void publish() {
		if(!cap_.isOpened()) {
			RCLCPP_ERROR(this->get_logger(),
			             "Video capture not initialized");
			return;
		}

		cv::Mat frame;
		cap_ >> frame;

		if(frame.empty()) {
			// 如果帧为空, 重置视频播放到开头
			cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
			frame_counter_ = 0;
			cap_ >> frame;
		}

		// 将 OpenCV Mat 转为 ROS 消息
		auto msg =
		    cv_bridge::CvImage(std_msgs::msg::Header(),
		                       "bgr8", frame)
		        .toImageMsg();
		publisher_->publish(*msg);

		frame_counter_++;
	}

	rclcpp::Publisher<Image>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	cv::VideoCapture cap_;
	int frame_counter_;
	int total_frames_;
};


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StreamNode>());
	rclcpp::shutdown();
	return 0;
}
