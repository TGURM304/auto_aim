#include "lights.hpp"

#include <numbers>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <utility>


std::pair<Light, LightCriterion> Light::from_contour(
    const Contour& contour, const cv::Mat& img) {
	using namespace std;
	using namespace std::numbers;
	using namespace cv;

	Moments m = cv::moments(contour);
	if(m.m00 < 1e-6)
		return make_pair(
		    Light(), LightCriterion{.one_vote_no = true});

	// 几何计算
	double delta_root = sqrtf(4 * m.mu11 * m.mu11
	                          + powf(m.mu20 - m.mu02, 2));
	double A = m.mu20 - m.mu02 + delta_root;

	double theta = pi / 2;
	double x = 0, y = 1;
	if(fabs(A) > 1e-6) {
		double B = 2 * m.mu11 / A;
		x = 1. / sqrtf(B * B + 1.);
		y = B * x;
		theta = atanf(B);
	}

	double var_pri =
	    .5 * (m.mu20 + m.mu02 + delta_root) / m.m00;
	double len_half_pri = sqrtf(3 * var_pri);

	double var_sec =
	    .5 * (m.mu20 + m.mu02 - delta_root) / m.m00;
	double len_half_sec = sqrtf(3 * var_sec);

	Vec2d dir_v = Vec2d(x, y);
	Vec2d dir_sec_v = Vec2d(-y, x);
	Vec2d pos_v = Vec2d(m.m10 / m.m00, m.m01 / m.m00);

	// 检查颜色
	Vec3i color = {0, 0, 0};
	double x_list[] = {-1.1, -1., 1., 1.1};
	double y_list[] = {-0.5, -0.3, 0., 0.3, 0.5};
	for(double x: x_list) {
		for(double y: y_list) {
			auto pos = pos_v + x * len_half_sec * dir_sec_v
			    + y * len_half_pri * dir_v;
			color +=
			    img.at<Vec3i>((int)pos[1], (int)pos[0]);
		}
	}
	double red_blue_ratio =
	    // 加 1 防止出现除 0 错误
	    (double)(color[2] + 1) / (color[0] + 1);

	return make_pair(
	    Light{.pos = pos_v,
	          .offset = len_half_pri * dir_v,
	          .angle = theta > 0 ? theta : -theta,
	          .length = 2 * len_half_pri,
	          .width = 2 * len_half_sec,
	          .color = red_blue_ratio > 1.
	              ? ArmorColor::RED
	              : ArmorColor::BLUE},
	    LightCriterion{
	        .one_vote_no = false,
	        .aspect_ratio = len_half_pri / len_half_sec,
	        .area = m.m00});
}

std::optional<Light> Light::try_from_contour(
    const Contour& contour, const cv::Mat& img,
    bool (*check)(const LightCriterion&)) {
	auto ans = Light::from_contour(contour, img);
	return check(ans.second) ? std::make_optional(ans.first)
	                         : std::nullopt;
}
