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

    // 分配内存, 保存RGB数据
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
    return camera_;
}

void showText(cv::Mat& frame, const std::string& msg) {
    printf("%s\n", msg.c_str());
    cv::Point position(20, 240);
    cv::putText(frame, msg, position,
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 0, 255), 2);
}

cv::Mat MindVision::getFrame(int hCamera) {
    if(hCamera == -1) {
        printf("相机初始化失败,Coding=-1\n");
        cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
        std::string text = "Camera initialization failed, error code: -1";
        showText(frame, text);
        return frame;
    } else if(hCamera == -2) {
        cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
        std::string text = "Camera initialization failed, error code: -2";
        showText(frame, text);
        return frame;
    } else {
        tSdkFrameHead sFrameInfo;
        BYTE* pbyBuffer;
        
        // 获取图像数据
        CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000);
        CameraImageProcess(hCamera, pbyBuffer, rgb_buffer_, &sFrameInfo);
        
        // 创建 cv::Mat 对象来处理图像数据
        cv::Mat frame(
            sFrameInfo.iHeight,  // 高度
            sFrameInfo.iWidth,   // 宽度
            sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8
                ? CV_8UC1    // 单通道
                : CV_8UC3,   // 三通道
            rgb_buffer_         // 图像数据缓冲区
        );
        // 释放图像缓冲区
        CameraReleaseImageBuffer(hCamera, pbyBuffer);

        return frame;
    }
}
