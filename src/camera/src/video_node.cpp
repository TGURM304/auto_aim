#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

class StreamNode: public rclcpp::Node {
public:
	StreamNode(): Node("stream_node") {
		publisher_ =
		    this->create_publisher<sensor_msgs::msg::Image>(
		        "camera/stream", 10);
		timer_ = this->create_wall_timer(
		    std::chrono::milliseconds(
		        25), // 设置为适当的帧率
		    std::bind(&StreamNode::publish, this));

		// 打开视频文件
		cap.open("./1.mp4");
		if(!cap.isOpened()) {
			RCLCPP_ERROR(this->get_logger(),
			             "Failed to open video file");
			return;
		}

		// 获取视频帧总数
		total_frames_ = static_cast<int>(
		    cap.get(cv::CAP_PROP_FRAME_COUNT));
		frame_counter_ = 0;
	}

private:
	void publish() {
		if(!cap.isOpened()) {
			RCLCPP_ERROR(this->get_logger(),
			             "Video capture not initialized");
			return;
		}

		cv::Mat frame;
		cap >> frame; // 从视频流中获取下一帧

		if(frame.empty()) {
			// 如果帧为空，重置视频播放到开头
			cap.set(cv::CAP_PROP_POS_FRAMES, 0);
			frame_counter_ = 0;
			cap >> frame; // 重新获取第一帧
		}

		// 将OpenCV Mat转换为ROS消息
		auto msg =
		    cv_bridge::CvImage(std_msgs::msg::Header(),
		                       "bgr8", frame)
		        .toImageMsg();
		publisher_->publish(*msg);

		frame_counter_++;
	}

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
	    publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	cv::VideoCapture cap;
	int frame_counter_;
	int total_frames_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StreamNode>());
	rclcpp::shutdown();
	return 0;
}
