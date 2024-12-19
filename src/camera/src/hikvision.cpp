#include "hikvision.hpp"
#include <opencv2/highgui.hpp>


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
    auto hk_config = config["mindvision"];
    exposuretime = hk_config["exposure_time"].value_or(30);
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
}

int HikVision::init(){
    // 初始化SDK
	nRet = MV_CC_Initialize();
    memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

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
        i++;
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
    nRet = MV_CC_SetEnumValue(camera_handle, "TriggerMode", 0);
    if (nRet != MV_OK){
        printf("MV_CC_SetTriggerMode fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    // 设置采集模式为连续采集
    nRet = MV_CC_SetEnumValue(camera_handle, "AcquisitionMode", 2);
    if (nRet != MV_OK){
        printf("MV_CC_SetAcquisitionMode fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    // 设置像素格式
    nRet = MV_CC_SetPixelFormat(camera_handle, PixelType_Gvsp_BGR8_Packed);
    if(nRet != MV_OK){
        printf("MV_CC_SetPixelFormat fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    // 设置曝光
    nRet = MV_CC_SetFloatValue(camera_handle, "ExposureTime", 1000 * exposuretime);

    // 设置Gamma
    nRet = MV_CC_SetGammaSelector(camera_handle, 10000);

    // 设置亮度
    nRet = MV_CC_SetBrightness(camera_handle, 10000);

    // TODO: HK更多参数设置

    // 开始取流
    nRet = MV_CC_StartGrabbing(camera_handle);
    if (nRet != MV_OK){
        printf("MV_CC_StartGrabbing fail! nRet [0x%x]\n", nRet);
        return nRet;
    }
    return MV_OK;
}

std::pair<cv::Mat, int> HikVision::getFrame() {
    nRet = MV_CC_GetImageBuffer(camera_handle, &frameOut, 1000);
    if (nRet != MV_OK) {
        cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
        showText(frame,decimalTohex(nRet));
        return std::make_pair(frame, nRet);
    }
    cv::Mat frame = cv::Mat(frameOut.stFrameInfo.nHeight, frameOut.stFrameInfo.nWidth, CV_8UC3, frameOut.pBufAddr); 
    nRet = MV_CC_FreeImageBuffer(camera_handle, &frameOut);
    if (nRet != MV_OK) {
        cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3) * 255;
        showText(frame,decimalTohex(nRet));
        return std::make_pair(frame, nRet);
    }

    return std::make_pair(frame, MV_OK);
}
