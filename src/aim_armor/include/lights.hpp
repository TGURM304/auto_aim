#ifndef _AIMARMOR_H_LIGHTS_
#define _AIMARMOR_H_LIGHTS_

#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <utility>
#include <vector>

using namespace cv;
using Contour = std::vector<Point>;


enum ArmorColor {
	RED,
	BLUE,
};


/**
 * @brief 灯条判据
 *
 * 使用内部的信息能够判断是否为有效灯条
 */
struct LightCriterion {
	/// @brief 一票否决
	/// 当计算过程出现错误等证明此灯条绝对无效的情况时此项为 `true`.
	/// 此时其余一切数据都不能相信, 包括灯条信息与其他判据.
	bool one_vote_no;

	/// @brief 长宽比
	double aspect_ratio;

	/// @brief 面积
	double area;
};
// TODO: 全局的判断器


struct Light {
	/// @brief 质心坐标
	Vec2d pos;

	/// @brief 偏移量
	/// pos +- offset 即为端点
	Vec2d offset;

	/// @brief 偏角. 弧度制
	double angle;

	/// @brief 长度
	double length;

	/// @brief 宽度
	double width;

	/// @brief 颜色
	ArmorColor color;

	/**
	 * @brief 获取灯条的两端点坐标
	 *
	 * 分别是两短边的中点. 不保证顺序.
	 */
	std::pair<Vec2d, Vec2d> points() {
		return std::make_pair(pos + offset, pos - offset);
	}

	/**
	 * @brief 获取灯条以整型储存的端点坐标
	 *
	 * 分别是两短边的中点. 不保证顺序.
	 */
	std::pair<Point2i, Point2i> int_points() {
		return std::make_pair(Point2i(pos + offset), Point2i(pos - offset));
	}

	/**
	 * @brief 获取完整装甲板边尺寸的端点坐标
	 *
	 * 灯条稍短于装甲板边, 补上这个差值.
	 *
	 * @warning 不保证绝对精确到装甲板边缘
	 */
	std::pair<Vec2d, Vec2d> full_points() {
		double scale = 2.0;
		return std::make_pair(pos + scale * offset, pos - scale * offset);
	}

	/**
	 * @brief 根据轮廓信息产生灯条, 并附上判据
	 *
	 * @param contour 轮廓
	 * @param img 图片, 用于判断颜色
	 * @return 返回一个元组 `(灯条, 检查依据)`
	 *
	 * 可通过判据检查是否是有效的灯条.
	 */
	static std::pair<Light, LightCriterion> from_contour(const Contour& contour,
	                                                     const cv::Mat& img);

	/**
	 * @brief 尝试从轮廓生成灯条
	 *
	 * @param contour 轮廓
	 * @param img 图片, 用于判断颜色
	 * @param check 检查函数. 此函数接受一个判据, 并以此判断是否是有效的灯条.
	 */
	static std::optional<Light> try_from_contour(
	    const Contour& contour, const cv::Mat& img,
	    std::function<bool(const LightCriterion&)> check);
};


#endif
