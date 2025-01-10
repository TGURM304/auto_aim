#include <algorithm>
#include <chrono>
#include <cstdint>

#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <interfaces/msg/armor.hpp>
#include <interfaces/msg/armors.hpp>
#include <interfaces/msg/target.hpp>
#include <tuple>
#include <vector>

#include "armors.hpp"


using ArmorsMsg = interfaces::msg::Armors;
using ArmorMsg = interfaces::msg::Armor;
using TargetMsg = interfaces::msg::Target;


class Tracker {
public:
	// 状态
	enum State { SEARCHING, TRACKING };

	// 打击目标结构体（发送的数据）
	struct Target {
		uint8_t aim_mode;
		float pitch_angle;
		float yaw_angle;
		float distance;
	} target;

	Armor rarmor;

	std::vector<Armor> armors;

public:
	Tracker() {
		target.aim_mode = 0;
		target.pitch_angle = .0;
		target.yaw_angle = .0;
	};

	~Tracker(){};

	/**
	* @brief 计算方向角和距离
	* 
	* @param pos 在相机坐标系下目标的位置向量
	* @return 返回一个元组, 为 (pitch 俯仰, yaw 偏航, 距离)
	*/
	std::tuple<float, float, float> calcPitchYawDist(cv::Vec3f pos) {
		using namespace std;
		float n = cv::norm(pos);
		auto v = pos / n;
		return make_tuple(-asin(v[1]), -asin(v[0] / sqrt(1 - v[1] * v[1])), n);
	}

	Target tracking() {
		if(armors.size() == 0) {
			target.aim_mode = 'a';
			return target;
		} else {
			target.aim_mode = 'a';
			std::tie(target.pitch_angle, target.yaw_angle, target.distance) =
			    calcPitchYawDist(armors[0].pos);
			target.pitch_angle = target.pitch_angle / std::numbers::pi * 180,
			target.yaw_angle = target.yaw_angle / std::numbers::pi * 180;
			armors.clear();
			return target;
		}
		// switch(currentState) {
		// case SEARCHING: {
		// 	// 如果未找到装甲板
		// 	if(isNotFindArmors()) {
		// 		target.aim_mode = 0;
		// 		target.pitch_angle = 0;
		// 		target.yaw_angle = 0;
		// 		break;
		// 	} else {
		// 		float mindistance = 2000000;
		// 		// for(auto armor: armors) {
		// 		// 	int yz_2 = (armor.pos[1] + armor.pos[2]) / 2;
		// 		// 	if(yz_2 < mindistance) {
		// 		// 		mindistance = yz_2;
		// 		// 		currentArmor.distance = yz_2;
		// 		// 		currentArmor.classes = armor.classes;
		// 		// 		target.aim_mode = 'a';
		// 		// 		std::tie(target.pitch_angle, target.yaw_angle, target.distance) = calcPitchYawDist(armor.pos);
		// 		// 	}
		// 		// }
		// 		// target.aim_mode = 'a';

		// 		std::tie(target.pitch_angle, target.yaw_angle, target.distance) = calcPitchYawDist(armors[0].pos);
		// 		previousArmor = currentArmor;
		// 		currentState = TRACKING;
		// 	}
		// 	break;
		// }

		// case TRACKING: {
		// 	// 如果当前装甲板与之前的装甲板相同
		// 	if(isSameBoard(currentArmor)) {
		// 		target.aim_mode = 1;
		// 		target.pitch_angle = 0;
		// 		target.yaw_angle = 0;
		// 		previousArmor = currentArmor;
		// 	} else {
		// 		// 如果不是同一个装甲板，切换到搜索模式
		// 		currentState = SEARCHING;
		// 		target.aim_mode = 0;
		// 		target.pitch_angle = 0;
		// 		target.yaw_angle = 0;
		// 	}
		// 	break;
		// }
		// default:
		// 	currentState = SEARCHING;
		// }
		// return target;
	}

private:
	// 当前跟踪状态
	State currentState = SEARCHING;
	// 跟踪的装甲板
	struct TrackedArmor {
		ArmorClasses classes;
		float distance;
	} previousArmor, currentArmor;

	// 是否找到装甲板
	bool isNotFindArmors() {
		return armors.empty();
	}

	bool isSameBoard(TrackedArmor &armor) {
		// TODO: 更改目标跟踪的算法,目前是非常弱智的小学生写法
		float distanceThreshold = 99999; // 距离阈值
		// 比较装甲板的编号和距离
		if(armor.classes == previousArmor.classes) {
			if(std::abs(armor.distance - previousArmor.distance)
			   < distanceThreshold) {
				return true;
			}
			return true;
		}
		return false;
	}
};

class TargetSender: public rclcpp::Node {
public:
	TargetSender(Tracker &tracker):
	Node("tracker_sendtarget_node"), tracker_(tracker) {
		publisher = this->create_publisher<TargetMsg>("/target/armor", 10);
		timer = this->create_wall_timer(
		    std::chrono::milliseconds(1),
		    std::bind(&TargetSender::send_target, this));
	}

private:
	void send_target() {
		tracker_.tracking();
		if(tracker_.target.aim_mode != 0) {
			message.aim_mode = tracker_.target.aim_mode;
			message.pitch_angle = tracker_.target.pitch_angle;
			message.yaw_angle = tracker_.target.yaw_angle;
			message.distance = tracker_.target.distance;
			publisher->publish(message);
		}
	}
	TargetMsg message;
	Tracker &tracker_;
	rclcpp::Publisher<TargetMsg>::SharedPtr publisher;
	rclcpp::TimerBase::SharedPtr timer;
};


class ArmorsReceiver: public rclcpp::Node {
public:
	ArmorsReceiver(Tracker &tracker):
	Node("tracker_getarmors_node"), tracker_(tracker) {
		subscription = this->create_subscription<ArmorsMsg>(
		    "/detector/armors", 10,
		    std::bind(&ArmorsReceiver::receiver, this, std::placeholders::_1));
	}

private:
	void receiver(const ArmorsMsg::SharedPtr armors_msg) {
		for(auto armor: armors_msg->armors) {
			// std::cout << armor.ori.x << std::endl;
			tracker_.rarmor.pos =
			    cv::Vec3d(armor.pos.x, armor.pos.y, armor.pos.z);
			tracker_.rarmor.ori =
			    cv::Vec3d(armor.ori.x, armor.ori.y, armor.ori.z);
			tracker_.rarmor.classes = (ArmorClasses)armor.classes;
			tracker_.armors.push_back(tracker_.rarmor);
		}
	}

private:
	Tracker &tracker_;
	ArmorMsg armor_msg;
	rclcpp::Subscription<ArmorsMsg>::SharedPtr subscription;
	rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	Tracker tracker;

	// 创建节点
	auto publisher_node = std::make_shared<TargetSender>(tracker);
	auto subscriber_node = std::make_shared<ArmorsReceiver>(tracker);

	// 多线程执行器
	rclcpp::executors::MultiThreadedExecutor executor;
	// 添加节点
	executor.add_node(publisher_node);
	executor.add_node(subscriber_node);
	// 启动
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
