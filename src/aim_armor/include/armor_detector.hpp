#ifndef _AIMARMOR_H_ARMORDETECTOR_
#define _AIMARMOR_H_ARMORDETECTOR_

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>

#include "toml.hpp"
#include "lights.hpp"
#include "armors.hpp"


class ArmorDetector {
public:
	ArmorDetector();

	~ArmorDetector();

	/**
	 * @brief 初始化装甲板检测类
	 *
	 * @return 返回错误码. 非 0 即为失败
	 * @retval -1 初始化失败: 解析配置文件失败
	 * @retval -2 其他错误
	 */
	int init();

	/**
	 * @brief 匹配装甲板
	 *
	 * @param armors 匹配的装甲板将会附加到此列表后面
	 * @param img    从此图片中匹配
	 * @param color  敌方颜色. 将只会匹配这个颜色的装甲板
	 * @return 返回匹配到的装甲板数量
	 */
	size_t match_armors(std::vector<Armor>& armors, const cv::Mat& img,
	                    ArmorColor color);

private:
	/**
	 * @brief 预处理图片
	 *
	 * 图片二值化并做开运算去噪
	 *
	 * @param out 输出图片
	 * @param in  输入图片
	 */
	void preprocess(cv::Mat& out, const cv::Mat& in);

	/**
	 * @brief 按顺序排列两个灯条的 4 个端点
	 *
	 * 按 (左下:-- , 左上:-+ , 右上:++ , 右下:+-) 的顺序排列
	 *
	 * @param l1 第一个灯条
	 * @param l2 第二个灯条
	 * @return 按顺序排列的点的坐标
	 */
	std::vector<cv::Point2d> sort_points(const Light& l1, const Light& l2);

	/**
	 * @brief 计算原始矩形的信息
	 *
	 * 所谓原始矩形, 是指这样一个在空间中的矩形: 它的 4 个顶点被拍摄为照片后与输入的 4 点一一对应.
	 *
	 * 计算前会根据矫正相机畸变.
	 *
	 * @warning `kpnts` 数组的长度必须等于 4
	 *
	 * @param kpnts 矩形四点在图中的坐标
	 *              必须按顺序排列.
	 *              参见 `ArmorDetector::sort_points`
	 * @return 返回一个判据, 可通过判据检查是否是有效的装甲板
	 */
	ArmorCriterion rect_info(const std::vector<cv::Point2d>& kpnts,
	                         const cv::Matx33d& camera,
	                         const cv::Matx<double, 1, 5>& dist);

	/**
	 * @brief 透视变换并二值化
	 *
	 * @note     此处不必去畸变, 因为收效甚微; 且要对某一片区域重映射, 消耗巨大
	 * @warning `kpnts` 数组的长度必须等于 4
	 *
	 * @param img   要处理的图片
	 * @param out   经过透视变换与二值化后的图片
	 * @param kpnts 要变换区域的 4 个顶点.
	 *              必须按顺序排列.
	 *              参见 `ArmorDetector::sort_points`
	 * @param size  输出图片的尺寸
	 */
	void perspective(const cv::Mat& img, cv::Mat& out,
	                 const std::vector<cv::Point2d>& kpnts, int size);

	/**
	 * @brief 对装甲板中心图案进行分类
	 *
	 * @param image 输入用于分类的图像. 图像尺寸为 64*64
	 * @return 装甲板类别
	 */
	std::string classify(const cv::Mat& image);

	/**
	 * @brief 尝试使用 PnP 解算计算目标装甲板在相机坐标系下的位置
	 *
	 * 计算前会根据矫正相机畸变.
	 *
	 * @param kpnts      装甲板灯条的 4 个顶点.
	 *                   必须按顺序排列.
	 *                   参见 `ArmorDetector::sort_points`
	 * @param armor_size 装甲板大小
	 * @return 返回一个元组: (装甲板中心的位置, 装甲板法向量).
	 *
	 * 上面说的都是在 相机坐标系 下. 法向量经过归一化.
	 * 装甲板法向量, 是指在装甲板正面方向的法向量. 所谓正面, 是指有图案的那一面.
	 */
	std::optional<std::pair<cv::Vec3d, cv::Vec3d>> pnp_solver(
	    const std::vector<cv::Point2d>& kpnts, ArmorSize armor_size,
	    const cv::Matx33d& camera, const cv::Matx<double, 1, 5>& dist);

private:
	LightCriterion light_cri[2];
	bool light_check(const LightCriterion& lc);
	ArmorCriterion armor_cri[2];
	bool armor_check(const ArmorCriterion& ac);

private:
	ov::Core core;
	ov::InferRequest infer_request;

	/// @brief 定义图像分类类别
	std::array<std::string, 8> classes;

	cv::Matx33d camera;
	cv::Matx<double, 1, 5> dist;
};


#endif
