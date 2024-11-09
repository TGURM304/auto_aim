#ifndef _CAMERA_H_MINDVISION_
#define _CAMERA_H_MINDVISION_

#include <opencv2/opencv.hpp>

#include <MVSDK/CameraApi.h>


class MindVision {
public:
	MindVision();

	~MindVision();

	/**
	 * @brief 初始化相机
	 *
	 * @param channel : 1 为 MONO8, 其他为 BGR8
	 * @retval -1 没有连接设备
	 * @retval -2 初始化失败
	 */
	int init(int channel);

	/**
	 * @brief 检查相机是否初始化成功
	 * @retval -1 没有连接设备
	 * @retval -2 初始化失败
	 */
	int isSuccessfulInit();

	/**
	 * @brief 获取帧
	 *
	 * 若相机初始化失败, 则返回 480*640 大小的错误描述图像.
	 *
	 * @retval -1 没有连接设备
	 * @retval -2 初始化失败
	 */
	cv::Mat getFrame();


private:
	/// @brief 处理后数据缓存区
	unsigned char* rgb_buffer_;
	/// @brief 相机计数
	int camera_cnt_ = 1;
	int status_ = -1;
	/// @brief 相机的设备信息
	tSdkCameraDevInfo camera_enum_list_;
	/// @brief 相机的操作句柄
	CameraHandle camera_;
	/// @brief 设备描述信息
	tSdkCameraCapbility capability_;
};

#endif
