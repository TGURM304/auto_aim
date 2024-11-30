#ifndef _AIMARMOR_H_ARMORDETECTOR_
#define _AIMARMOR_H_ARMORDETECTOR_

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <toml.hpp>
#include <string>


class ArmorDetector {
public:
	ArmorDetector();

	~ArmorDetector();

	/**
	 * @brief 初始化装甲板检测类
	 *
	 * @retval 1 初始化成功
	 * @retval -1 初始化失败
	 */
	int init();

	/**
	 * @brief 对装甲板中心图案进行分类
	 *
	 * @param image 输入用于分类的图像
	 * @return 数据类别
	 */
	std::string classify(cv::Mat image);

private:
	/// @brief 初始化配置文件
	toml::table config =
	    toml::parse_file("./assets/config.toml");
	/// @brief OpenVINO
	ov::Core core;
	ov::InferRequest infer_request;
	/// @brief 定义图像分类类别
	std::array<std::string, 8> classes;
};

#endif
