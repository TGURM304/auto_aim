#include "../dependencies/MVSDK/include/CameraApi.h" // 相机SDK的API头文件
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>

using namespace cv;

unsigned char *g_pRgbBuffer;  // 处理后数据缓存区

int main()
{
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; // 设备描述信息
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;
    int channel = 3;

    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);
    printf("count = %d\n", iCameraCounts);

    // 没有连接设备
    if (iCameraCounts == 0) {
        return -1;
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

    // 初始化失败
    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS) {
        return -1;
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);

    // 分配内存，保存RGB数据
    g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);

    // 让SDK进入工作模式，开始接收来自相机发送的图像数据
    CameraPlay(hCamera);

    // 配置相机输出格式
    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    } else {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }
 
    // 循环显示10000帧图像
    while (1) {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer,0) == CAMERA_STATUS_SUCCESS) {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            // 使用 cv::Mat 替代 IplImage
            cv::Mat matImage(
                sFrameInfo.iHeight, // 高度
                sFrameInfo.iWidth,  // 宽度
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3, // 类型：单通道或者三通道
                g_pRgbBuffer        // 数据缓冲区
            );

            // 显示图像
            imshow("Opencv Demo", matImage);
            waitKey(5);

            // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }
    }

    // 反初始化相机
    CameraUnInit(hCamera);

    // 释放分配的内存
    free(g_pRgbBuffer);

    return 0;
}
