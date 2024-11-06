#ifndef _CAMERA_H_MINDVISION_
#define _CAMERA_H_MINDVISION_

#include <opencv2/opencv.hpp>

#include <MVSDK/CameraApi.h>


// TODO: 注释
class MindVision {
public:
	MindVision();

	~MindVision();

	int Init(int channel);

	cv::Mat getMindvision(int hCamera);


private:
	// 处理后数据缓存区
	unsigned char *g_pRgbBuffer;

	int iCameraCounts = 1;
	int iStatus = -1;
	tSdkCameraDevInfo tCameraEnumList;
	int hCamera;
	tSdkCameraCapbility tCapability; // 设备描述信息
};

#endif
