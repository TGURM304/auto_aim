#include "hikvision.hpp"
#include <opencv2/highgui.hpp>
#include "MvCameraControl.h"
#include "MvObsoleteInterfaces.h"


static void showText(cv::Mat& frame, const std::string& msg) {
	cv::Point position(20, 240);
	cv::putText(frame, msg, position, cv::FONT_HERSHEY_SIMPLEX, 1.0,
	            cv::Scalar(0, 0, 255), 2);
}


static std::string decimalTohex(int num) {
	std::stringstream ss;
	ss << "0x" << std::hex << num; // 将整数转换为十六进制
	return ss.str();               // 返回十六进制字符串
}


HikVision::HikVision() {
	auto hk_config = config["hikvision"];
	exposuretime = hk_config["exposure_time"].value_or(30);
	autoexposure = hk_config["auto_exposure"].value_or(false);
	brightness = hk_config["brightness"].value_or(100);
	autobalance = hk_config["auto_whitebalance"].value_or(true);
	printf("HikVision Start\n");
}

HikVision::~HikVision() {
	// 停止取流
	MV_CC_StopGrabbing(camera_handle);
	// 关闭设备
	MV_CC_CloseDevice(camera_handle);
	// 销毁句柄
	MV_CC_DestroyHandle(camera_handle);
	// 反初始化SDK
	MV_CC_Finalize();
}

int HikVision::init() {
	pDataForRGB = (unsigned char*) malloc(1440 * 1080 * 4 + 2048);
	
	// 初始化SDK
	nRet = MV_CC_Initialize();
	memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
	if(nRet != MV_OK) {
		printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	pstCvtParam.enSrcPixelType = PixelType_Gvsp_BayerRG8;
	pstCvtParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

	int i = 1;
	do {
		nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
		printf("第%d次尝试获取Hik相机列表\n", i);
		if(nRet != MV_OK) {
			printf("MV_CC_EnumDevices fail! nRet [0x%x]\n", nRet);
		}
		i++;
	} while(device_list.nDeviceNum == 0 && i <= 10);

	// 创建句柄
	nRet = MV_CC_CreateHandle(&camera_handle, device_list.pDeviceInfo[0]);
	if(nRet != MV_OK) {
		printf("MV_CC_CreateHandle fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	// 打开相机
	nRet = MV_CC_OpenDevice(camera_handle);
	if(nRet != MV_OK) {
		printf("MV_CC_OpenDevice fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	// 设置相机传输速率
	nRet = MV_USB_SetTransferSize(camera_handle, 0x2000000);
	if(nRet != MV_OK) {
		printf("MV_CC_SetTransferSize fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	// 设置触发模式为off
	nRet = MV_CC_SetEnumValue(camera_handle, "TriggerMode", 0);
	if(nRet != MV_OK) {
		printf("MV_CC_SetTriggerMode fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	// 设置采集模式为连续采集
	nRet = MV_CC_SetEnumValue(camera_handle, "AcquisitionMode", 2);
	if(nRet != MV_OK) {
		printf("MV_CC_SetAcquisitionMode fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	// 设置为8bit位深
	nRet = MV_CC_SetEnumValue(camera_handle, "ADCBitDepth", 0);
	if(nRet != MV_OK) {
		printf("MV_CC_SetADCBitDepth fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	nRet = MV_CC_SetEnumValue(camera_handle, "PixelFormat",
	                          PixelType_Gvsp_BayerRG8);
	if(nRet != MV_OK) {
		printf("MV_CC_SetPixelFormat fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	// 设置白平衡模式
	nRet = MV_CC_SetEnumValue(camera_handle, "BalanceWhiteAuto",
	                          autobalance ? 1 : 0);
	if(nRet != MV_OK) {
		printf("MV_CC_BalanceWhiteAuto fail! nRet [0x%x]\n", nRet);
		return nRet;
	}


	// 设置曝光模式
	nRet =
	    MV_CC_SetEnumValue(camera_handle, "ExposureAuto", autoexposure ? 1 : 0);
	if(nRet != MV_OK) {
		printf("MV_CC_SetExposureAuto fail! nRe`	 `t [0x%x]\n", nRet);
		return nRet;
	}

	if(autoexposure ? 1 : 0) { // 设置曝光
		nRet = MV_CC_SetFloatValue(camera_handle, "ExposureTime",
		                           1000 * exposuretime);
		if(nRet != MV_OK) {
			printf("MV_CC_SetExposureTime fail! nRet [0x%x]\n", nRet);
			return nRet;
		}
	}

	// 增益设置
	nRet = MV_CC_SetFloatValue(camera_handle, "Gain", 0);
	if(nRet != MV_OK) {
		printf("MV_CC_SetGain fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	// 插值算法设置
	nRet = MV_CC_SetBayerCvtQuality(camera_handle, 1);
	if(nRet != MV_OK) {
		printf("MV_CC_SetBayerCvtQuality fail! nRet [0x%x]\n", nRet);
		return nRet;
	}

	// ch:获取数据包大小 | en:Get payload size
	MVCC_INTVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_INTVALUE));
	nRet = MV_CC_GetIntValue(camera_handle, "PayloadSize", &stParam);
	if(MV_OK != nRet) {
		printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
	}

	// 开始取流
	nRet = MV_CC_StartGrabbing(camera_handle);
	if(nRet != MV_OK) {
		printf("MV_CC_StartGrabbing fail! nRet [0x%x]\n", nRet);
		return nRet;
	}
	return MV_OK;
}

std::pair<cv::Mat, int> HikVision::getFrame() {
	nRet = MV_CC_GetImageBuffer(camera_handle, &frameOut, 1000);
	if(nRet != MV_OK) {
		cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
		showText(frame, decimalTohex(nRet));
		return std::make_pair(frame, nRet);
	}

	// pDataForRGB = (unsigned char*)malloc(
	//     frameOut.stFrameInfo.nWidth * frameOut.stFrameInfo.nHeight * 4 + 2048);
	
	if(NULL == pDataForRGB) {
		cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
		nRet = 0x80000108;
		showText(frame, decimalTohex(nRet));
		return std::make_pair(frame, nRet);
	}
	// 像素格式转换
	pstCvtParam = {0};
	// 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
	// 目标像素格式，输出数据缓存，提供的输出缓冲区大小
	pstCvtParam.nWidth = frameOut.stFrameInfo.nWidth;
	pstCvtParam.nHeight = frameOut.stFrameInfo.nHeight;
	pstCvtParam.pSrcData = frameOut.pBufAddr;
	pstCvtParam.nSrcDataLen = frameOut.stFrameInfo.nFrameLen;
	pstCvtParam.enSrcPixelType = PixelType_Gvsp_BayerRG8;
	pstCvtParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
	pstCvtParam.pDstBuffer = pDataForRGB;
	pstCvtParam.nDstBufferSize =
	    frameOut.stFrameInfo.nWidth * frameOut.stFrameInfo.nHeight * 4 + 2048;
	pstCvtParam.nDstLen =
	    frameOut.stFrameInfo.nWidth * frameOut.stFrameInfo.nHeight * 3;
	nRet = MV_CC_ConvertPixelTypeEx(camera_handle, &pstCvtParam);

	if(MV_OK != nRet) {
		printf("MV_CC_ConvertPixelTypeEx fail! nRet [%x]\n", nRet);
	}
	if(nRet != MV_OK) {
		cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
		showText(frame, decimalTohex(nRet));
		return std::make_pair(frame, nRet);
	}


	cv::Mat frame =
	    cv::Mat(frameOut.stFrameInfo.nHeight, frameOut.stFrameInfo.nWidth,
	            CV_8UC3, pDataForRGB);

	nRet = MV_CC_FreeImageBuffer(camera_handle, &frameOut);
	if(nRet != MV_OK) {
		cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
		showText(frame, decimalTohex(nRet));
		return std::make_pair(frame, nRet);
	}

	return std::make_pair(frame, MV_OK);
}
