#include "toml.hpp"
#include "MvCameraControl.h"

#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>

class HikVision {
public:
    HikVision();

    ~HikVision();

    /**
     * @brief 初始化HikSDK并准备获取帧
     * 
     * @retval nRet MvErrorDefine.h中的错误码
     * @retval MV_OK 初始化完成
     * @retval -1 pData内存分配错误
     * @retval -2 未连接到相机
     */
    int init();

    /**
     * @brief 从相机获取一帧图像
     * 
     * @return std::pair<cv::Mat, int> 返回一帧及其状态码 
     */
    std::pair<cv::Mat, int> getFrame();

private:
    /// @brief 状态码
    int nRet = MV_OK;
    /// @brief 相机句柄;
    void * camera_handle = NULL;
    /// @brief 相机列表;
    MV_CC_DEVICE_INFO_LIST device_list;
    /// @brief 相机信息
    MV_IMAGE_BASIC_INFO img_info;
    /// @brief 图像帧相关信息
    MV_FRAME_OUT_INFO_EX stImageInfo;
    /// @brief 图像数据接收指针
    // unsigned char* pData = NULL;
    /// @brief 图像结构体，图像地址及图像信息
    MV_FRAME_OUT frameOut;
private:
    /// @brief 配置文件
    toml::table config = toml::parse_file("./assets/config.toml");
    /// @brief 曝光时间
    int exposuretime = 10;

};