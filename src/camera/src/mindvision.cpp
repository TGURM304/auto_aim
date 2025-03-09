#include <chrono>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mindvision.hpp"


static const rclcpp::Logger LOGGER = rclcpp::get_logger("mindvision");


MindVision::MindVision() {
	RCLCPP_INFO(LOGGER, "MindVision Start\n");
}

MindVision::~MindVision() {
	// 反初始化相机
	CameraUnInit(camera_);
	// 释放分配的内存
	free(rgb_buffer_);
}

int MindVision::init(int channel = 2) {
	// 1 表示中文
	CameraSdkInit(1);

	// 枚举设备, 并建立设备列表
	status_ = CameraEnumerateDevice(&camera_enum_list_, &camera_cnt_);
	RCLCPP_INFO(LOGGER, "state = %d\ncount = %d\n", status_, camera_cnt_);
	// 没有连接设备
	if(camera_cnt_ == 0) {
		return camera_ = -1;
	}

	// 相机初始化
	// 初始化成功后, 才能调用任何其他相机相关的操作接口
	status_ = CameraInit(&camera_enum_list_, -1, -1, &camera_);

	// 初始化失败
	RCLCPP_INFO(LOGGER, "state = %d\n", status_);
	if(status_ != CAMERA_STATUS_SUCCESS) {
		return camera_ = -2;
	}

	// 获得相机的特性描述结构体
	CameraGetCapability(camera_, &capability_);

	// 分配内存, 保存 RGB 数据
	rgb_buffer_ = (unsigned char*)malloc(640 * 480 * 3);

	// 获得相机的特性描述结构体
	CameraGetCapability(camera_, &capability_);

	// 获取配置
	uint8_t aestate =
	    this->declare_parameter("mindvision.auto_exposure", false) ? 1 : 0;
	uint16_t exposuretime =
	    this->declare_parameter("mindvision.exposure_time", 30);
	uint16_t gamma = this->declare_parameter("mindvision.gamma", 100);
	uint16_t contrast = this->declare_parameter("mindvision.contrast", 100);
	uint16_t saturation = this->declare_parameter("mindvision.saturation", 100);

	if(!aestate) {
		// 关闭自动曝光
		CameraSetAeState(camera_, 0);
		// 设置曝光时间
		CameraSetExposureTime(camera_, exposuretime * 1000);
	}

	// 设置gamma值
	CameraSetGamma(camera_, gamma);
	// 设置对比度
	CameraSetContrast(camera_, contrast);
	// 设置饱和度
	CameraSetSaturation(camera_, saturation);
	// 设置帧率ID, 0为60帧,1为108帧(?)
	CameraSetFrameSpeed(camera_, 1);

	// 进入工作模式, 开始接收来自相机发送的图像数据
	CameraPlay(camera_);

	// 配置相机输出格式
	if(channel == 1) {
		CameraSetIspOutFormat(camera_, CAMERA_MEDIA_TYPE_MONO8);
	} else {
		CameraSetIspOutFormat(camera_, CAMERA_MEDIA_TYPE_BGR8);
	}
	return 0;
}

int MindVision::getErrno() {
	return camera_ > 0 ? 0 : camera_;
}

static void showText(cv::Mat& frame, const std::string& msg) {
	printf("%s\n", msg.c_str());
	cv::Point position(20, 240);
	cv::putText(frame, msg, position, cv::FONT_HERSHEY_SIMPLEX, 1.0,
	            cv::Scalar(0, 0, 255), 2);
}

cv::Mat MindVision::getFrame() {
	if(getErrno()) {
		cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
		const char* msg = NULL;
		switch(getErrno()) {

		case -1:
			msg = "[-1] no device connected";
			break;
		case -2:
			msg = "[-2] camera initialization failed";
			break;
		default:
			msg = "[114514] Unknown Error";
		}

		showText(frame, msg);
		return frame;
	}

	tSdkFrameHead frame_info;
	rgb_buffer_ = CameraGetImageBufferEx(camera_, &frame_info.iWidth,
	                                     &frame_info.iHeight, 1000);
	cv::Mat frame(frame_info.iHeight, frame_info.iWidth, CV_8UC3, rgb_buffer_);
	return frame;
}

int MindVision::record(std::string fileSavePath, int time) {
	cv::Mat firstFrame = getFrame();
	if(firstFrame.empty()) {
		RCLCPP_ERROR(LOGGER, "Failed to get frame!\n");
		return -1;
	}

	int frame_width = firstFrame.cols;
	int frame_height = firstFrame.rows;

	cv::VideoWriter out(fileSavePath + "/output.avi",
	                    cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 108,
	                    cv::Size(frame_width, frame_height));

	if(!out.isOpened()) {
		RCLCPP_ERROR(LOGGER, "Failed to open video writer!\n");
		return -2;
	}

	auto startTime = std::chrono::steady_clock::now();
	auto duration = std::chrono::milliseconds(time);

	while(true) {
		cv::Mat frame = getFrame();

		if(frame.empty()) {
			RCLCPP_ERROR(LOGGER, "Failed to get a frame!\n");
			break;
		}

		// 大津法二值化
		// cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		// cv::threshold(frame, frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

		out.write(frame);
		cv::imshow("record", frame);
		cv::waitKey(1);
		if(std::chrono::steady_clock::now() - startTime >= duration) {
			break;
		}
	}

	out.release();
	cv::destroyAllWindows();
	return 1;
}
