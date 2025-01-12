#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageRecorder: public rclcpp::Node {
public:
	ImageRecorder(): Node("image_recorder") {
		image_subscription_ =
		    this->create_subscription<sensor_msgs::msg::Image>(
		        "/armors_image", 10,
		        std::bind(&ImageRecorder::image_callback, this,
		                  std::placeholders::_1));
		video_writer_initialized_ = false;
	}

private:
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
		cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

		if(!video_writer_initialized_) {
			int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
			video_writer_.open("./tmp/output.avi", fourcc, 110,
			                   cv::Size(frame.cols, frame.rows), true);

			if(!video_writer_.isOpened()) {
				RCLCPP_ERROR(this->get_logger(), "打开视频写入失败");
				return;
			}

			video_writer_initialized_ = true;
		}

		if(!frame.empty()) {
			video_writer_.write(frame);
		}
	}
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
	    image_subscription_;
	cv::VideoWriter video_writer_;
	bool video_writer_initialized_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageRecorder>());
	rclcpp::shutdown();
	return 0;
}
