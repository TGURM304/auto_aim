#ifndef _AIMARMOR_H_ARMORS_
#define _AIMARMOR_H_ARMORS_


#define BIG_ARMOR_HEIGHT   5.5e-2  /* m */
#define BIG_ARMOR_WIDTH    22.8e-2 /* m */
#define SMALL_ARMOR_HEIGHT 5.5e-2  /* m */
#define SMALL_ARMOR_WIDTH  13.5e-2 /* m */


enum ArmorSize {
	BIG,
	SMALL,
};

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
	// TODO: Armor
};


#endif
