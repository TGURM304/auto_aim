#ifndef _AIMARMOR_H_ARMORS_
#define _AIMARMOR_H_ARMORS_

#include <opencv2/core/matx.hpp>
#include <string>


#define BIG_ARMOR_HEIGHT   5.5e-2  /* m */
#define BIG_ARMOR_WIDTH    22.8e-2 /* m */
#define SMALL_ARMOR_HEIGHT 5.5e-2  /* m */
#define SMALL_ARMOR_WIDTH  13.5e-2 /* m */


enum ArmorSize {
	BIG,
	SMALL,
};


enum ArmorClasses { NUM1, NUM2, NUM3, NUM4, BASE, QSZ, SB, NONE };

/**
 * @brief 将描述装甲板种类的字符串转为枚举值
 */
inline ArmorClasses str_to_classes(const std::string& str) {
	if(str == "1")
		return NUM1;
	else if(str == "2")
		return NUM2;
	else if(str == "3")
		return NUM3;
	else if(str == "4")
		return NUM4;
	else if(str == "base")
		return BASE;
	else if(str == "qsz")
		return QSZ;
	else if(str == "sb")
		return SB;
	else
		return NONE;
}


/**
 * @brief 装甲板判据 (仅为几何视角)
 *
 * 使用内部的信息能够判断是否为有效装甲板,
 * 并且获得装甲板的其他信息.
 *
 * @warning 此判据只是能构成装甲板的必要条件.
 * 也就是说, 即使通过, 也不能说明一定可以构成装甲板.
 * 还需进一步验证, 如识别中心图像 (数字).
 */
struct ArmorCriterion {
	/// @brief 一票否决
	///
	/// 当计算过程出现错误等证明此灯条绝对无效的情况时此项为 `true`.
	/// 此时其余一切数据都不能相信, 包括灯条信息与其他判据.
	bool one_vote_no;

	/// @brief 重建后平行四边形的长宽比
	///
	/// 始终大于 1
	double aspect_ratio;

	/// @brief 重建后平行四边形的邻边夹角
	///
	/// [0, 180)
	double edge_angle;

	/// @brief 原始 (图像中) 的对边 (两灯条) 间夹角
	///
	/// [0, 90)
	double original_angle;
};


struct Armor {
	/// @brief 装甲板中心图案的类别
	ArmorClasses classes;

	/// @brief 装甲板的位置
	///
	/// 在相机坐标系下
	cv::Vec3d pos;

	/// @brief 装甲板朝向
	///
	/// 即装甲板的法向量
	cv::Vec3d ori;
};


#endif
