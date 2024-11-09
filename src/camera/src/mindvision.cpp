#include "mindvision.hpp"

MindVision::MindVision() {
    printf("MindVision Start\n");
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
    status_ = CameraEnumerateDevice(&camera_enum_list_,
                                    &camera_cnt_);
    printf("state = %d\ncount = %d\n", status_,
           camera_cnt_);

    // 没有连接设备
    if(camera_cnt_ == 0) {
        return -1;
    }

    // 相机初始化
    // 初始化成功后, 才能调用任何其他相机相关的操作接口
    status_ =
        CameraInit(&camera_enum_list_, -1, -1, &camera_);

    // 初始化失败
    printf("state = %d\n", status_);
    if(status_ != CAMERA_STATUS_SUCCESS) {
        return -2;
    }

    // 获得相机的特性描述结构体
    CameraGetCapability(camera_, &capability_);

	// 分配内存, 保存 RGB 数据
	rgb_buffer_ = (unsigned char*)malloc(
	    capability_.sResolutionRange.iHeightMax
	    * capability_.sResolutionRange.iWidthMax * 3);

    // 进入工作模式, 开始接收来自相机发送的图像数据
    CameraPlay(camera_);

	// 配置相机输出格式
	if(channel == 1) {
		CameraSetIspOutFormat(camera_,
		                      CAMERA_MEDIA_TYPE_MONO8);
	} else {
		CameraSetIspOutFormat(camera_,
		                      CAMERA_MEDIA_TYPE_BGR8);
	}
	return 0;
}

int MindVision::isSuccessfulInit() {
	return camera_;
}

void showText(cv::Mat& frame, const std::string& msg) {
    printf("%s\n", msg.c_str());
    cv::Point position(20, 240);
    cv::putText(frame, msg, position,
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 0, 255), 2);
}

cv::Mat MindVision::getFrame() {
	if(!isSuccessfulInit()) {
		cv::Mat frame =
		    cv::Mat::ones(480, 640, CV_8UC3) * 255;
		const char* msg = NULL;

		switch(isSuccessfulInit()) {
		case -1:
			msg = "[-1] no device connected";
			break;
		case -2:
			msg = "[-2] camera initialization failed";
			break;
		}

		showText(frame, msg);
		return frame;
	}

	tSdkFrameHead frame_info;
	BYTE* buffer;

	CameraGetImageBuffer(camera_, &frame_info, &buffer,
	                     1000);
	CameraImageProcess(camera_, buffer, rgb_buffer_,
	                   &frame_info);

	// 使用 cv::Mat 替代 IplImage
	cv::Mat frame(
	    frame_info.iHeight, frame_info.iWidth,
	    frame_info.uiMediaType == CAMERA_MEDIA_TYPE_MONO8
	        /* 单通道或者三通道 */
	        ? CV_8UC1
	        : CV_8UC3,
	    rgb_buffer_);

	// 在成功调用 CameraGetImageBuffer 后,
	// 必须调用 CameraReleaseImageBuffer 来释放获得的 buffer
	CameraReleaseImageBuffer(camera_, buffer);

	return frame;
}
