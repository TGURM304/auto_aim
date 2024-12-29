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
		timer_ =
		    this->create_wall_timer(5ms, std::bind(&StreamNode::publish, this));

		openVideoFile(VIDEO_FILE_PATH);
		if(!cap_.isOpened()) {
			RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
			return;
		}

		// 获取视频帧总数
		total_frames_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT));
		frame_counter_ = 0;
	}

private:
	// 视频文件路径常量
	const std::string VIDEO_FILE_PATH = "./1.avi";

	void openVideoFile(const std::string &file_path) {
		if(cap_.isOpened()) {
			cap_.release(); // 如果视频已经打开，先释放资源
		}

		cap_.open(file_path); // 重新打开视频文件
		if(!cap_.isOpened()) {
			RCLCPP_ERROR(this->get_logger(), "Unable to open video file: %s",
			             file_path.c_str());
		} else {
			RCLCPP_INFO(this->get_logger(),
			            "Video file opened successfully: %s",
			            file_path.c_str());
		}
	}

	void publish() {
		if(!cap_.isOpened()) {
			RCLCPP_ERROR(this->get_logger(), "Video capture not initialized");
			return;
		}

		cv::Mat frame;
		cap_ >> frame; // 读取当前帧

		if(frame.empty()) {
			// 如果读取的帧为空，表示视频已经播放完
			RCLCPP_INFO(this->get_logger(), "Video end reached. Restarting.");
			// 释放当前视频资源
			cap_.release();
			// 重新打开视频文件并重置计数器
			openVideoFile(VIDEO_FILE_PATH);
			frame_counter_ = 0;
			cap_ >> frame; // 重新读取第一帧
		}

		// 将 OpenCV Mat 转为 ROS 消息
		auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
		               .toImageMsg();
		publisher_->publish(*msg); // 发布消息

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
