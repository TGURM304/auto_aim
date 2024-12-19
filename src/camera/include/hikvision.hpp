#include "toml.hpp"
#include "MvCameraControl.h"

#include <opencv2/opencv.hpp>

class HikVision {
public:
    HikVision();

    ~HikVision();

    /**
     * @brief 初始化HikSDK并准备获取帧
     * 
     * @retval nRet MvErrorDefine.h中的错误码
     * @retval 1 初始化完成
     * @retval -1 pData内存分配错误
     * @retval -2 未连接到相机
     */
    int init();


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
    unsigned char* pData = NULL;
    /// @brief Hik特有的一个整数型参数值？
    MVCC_INTVALUE stParam = {0,0,0,0,0};
    /// @brief 存储从相机获取的图像数据的大小
    unsigned int nDataSize = stParam.nCurValue;
};