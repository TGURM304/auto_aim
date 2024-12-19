#include "hikvision.hpp"
#include <cstdio>
#include <cstdlib>
#include <opencv2/core/mat.hpp>
#include <string>
#include <thread>

static void showText(cv::Mat& frame, const std::string& msg) {
	cv::Point position(20, 240);
	cv::putText(frame, msg, position, cv::FONT_HERSHEY_SIMPLEX, 1.0,
	            cv::Scalar(0, 0, 255), 2);
}


static std::string decimalTohex(int num) {
    std::stringstream ss;
    ss << "0x" <<std::hex << num;  // 将整数转换为十六进制
    return ss.str();    // 返回十六进制字符串
}


HikVision::HikVision(){
    printf("HikVision Start\n");
}

HikVision::~HikVision(){
    // 停止取流
    MV_CC_StopGrabbing(camera_handle);
    // 关闭设备
    MV_CC_CloseDevice(camera_handle);
    // 销毁句柄
    MV_CC_DestroyHandle(camera_handle);
    // 反初始化SDK
    MV_CC_Finalize();
    // 释放内存
    free(pData);
}

int HikVision::init(){
    // 初始化SDK
	nRet = MV_CC_Initialize();

	if (nRet != MV_OK){
		printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
        return nRet;
	}
    int i = 1;
    do{
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        printf("第%d次尝试获取Hik相机列表\n", i);
        if (nRet != MV_OK){
            printf("MV_CC_EnumDevices fail! nRet [0x%x]\n", nRet);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
        ++i;
    }while(device_list.nDeviceNum == 0 && i <=10);


    // 创建句柄
    nRet = MV_CC_CreateHandle(&camera_handle, device_list.pDeviceInfo[0]);
    if (nRet != MV_OK){
        printf("MV_CC_CreateHandle fail! nRet [0x%x]\n", nRet);
        return nRet;
    }
    
    // 打开相机
    nRet = MV_CC_OpenDevice(camera_handle);
    if (nRet != MV_OK){
        printf("MV_CC_OpenDevice fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    // 设置触发模式为off
    // TODO: MV_CC_SetEnumValue是干吗的
    nRet = MV_CC_SetEnumValue(camera_handle, "TriggerMode", 0);
    if (nRet != MV_OK){
        printf("MV_CC_SetTriggerMode fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    // 开始取流
    nRet = MV_CC_StartGrabbing(camera_handle);
    if (nRet != MV_OK){
        printf("MV_CC_StartGrabbing fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    // 获取图像数据包的大小
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(camera_handle, "PayloadSize", &stParam);
    if (nRet != MV_OK){
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return nRet;
    }
    pData = (unsigned char*)malloc(stParam.nCurValue);
    if (pData == NULL){
        printf("Memory allocation failed\n");
        return -1;
    }
    return 1;
}

std::pair<cv::Mat, int> HikVision::getFrame() {
    if (nRet != MV_OK) {
        cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
        showText(frame,decimalTohex(nRet));
        return std::make_pair(frame, nRet);
    }
    nRet = MV_CC_GetOneFrameTimeout(camera_handle, pData, nDataSize, &stImageInfo, 1000);
    if (nRet != MV_OK) {
        cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
        showText(frame,decimalTohex(nRet));
        return std::make_pair(frame, nRet);
    }
    // test
    // printf("Get one frame: Width = %d, Height = %d, FrameNum = %d\n", stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
    cv::Mat frame(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
    return std::make_pair(frame, MV_OK);
}
