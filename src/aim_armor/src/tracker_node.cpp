#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <interfaces/msg/armor.hpp>
#include <interfaces/msg/armors.hpp>
#include <interfaces/msg/target.hpp>
#include <vector>


using ArmorsMsg = interfaces::msg::Armors;
using ArmorMsg = interfaces::msg::Armor;
using TargetMsg = interfaces::msg::Target;


class Tracker {
public:
	// 状态
	enum State { SEARCHING, TRACKING };
	// 装甲板类别
	enum Classes { NUM1, NUM2, NUM3, NUM4, BASE, QSZ, SB, NONE };

	// 打击目标结构体（发送的数据）
	struct Target {
		uint8_t aim_mode;
		float pitch_angle;
		float yaw_angle;
	} target;

	struct Armor {
		/// @brief 装甲板中心图案的类别
		std::string classes;
		/// @brief 装甲板的位置
		cv::Vec3d pos;
		/// @brief 装甲板朝向
		cv::Vec3d ori;

	} rarmor;

	std::vector<Armor> armors;

public:
	Tracker() {
		target.aim_mode = 0;
		target.pitch_angle = .0;
		target.yaw_angle = .0;
	};

	~Tracker(){};

	Target tracking() {
		switch(currentState) {
		case SEARCHING: {
			// 如果未找到装甲板
			if(isNotFindArmors()) {
				target.aim_mode = 0;
				target.pitch_angle = 0;
				target.yaw_angle = 0;
				break;
			} else {
				float mindistance = 2000000;
				for(auto armor: armors) {
					int yz_2 = (armor.pos[1]+armor.pos[2])/2;
					if(yz_2 < mindistance) {
						mindistance = yz_2;
						currentArmor.distance = yz_2;
						currentArmor.classes = numToClass(armor.classes);
						// TODO: 加入弹道解算
						target.aim_mode = 1;
						target.pitch_angle = 0;
						target.yaw_angle = 0;
					}
				}
				previousArmor = currentArmor;
				currentState = TRACKING;
			}
			break;
		}

		case TRACKING: {
			// 如果当前装甲板与之前的装甲板相同
			if(isSameBoard(currentArmor)) {
				target.aim_mode = 1;
				target.pitch_angle = 0;
				target.yaw_angle = 0;
				previousArmor = currentArmor;
			} else {
				// 如果不是同一个装甲板，切换到搜索模式
				currentState = SEARCHING;
				target.aim_mode = 0;
				target.pitch_angle = 0;
				target.yaw_angle = 0;
			}
			break;
		}
		default: {
			currentState = SEARCHING;
		}
		}
		return target;
	}

private:
	// 将number转换为Classes 枚举值
	Classes numToClass(const std::string &str) {
		if(str == "1")
			return NUM1;
		if(str == "2")
			return NUM2;
		if(str == "3")
			return NUM3;
		if(str == "4")
			return NUM4;
		if(str == "base")
			return BASE;
		if(str == "qsz")
			return QSZ;
		if(str == "sb")
			return SB;
		return NONE;
	}

private:
	// 当前跟踪状态
	State currentState = SEARCHING;
	// 跟踪的装甲板
	struct TrackedArmor {
		Classes classes;
		float distance;
	} previousArmor, currentArmor;

	// 是否找到装甲板
	bool isNotFindArmors() {
		return armors.empty();
	}

	bool isSameBoard(TrackedArmor &armor) {
		// TODO: 更改目标跟踪的算法,目前是非常弱智的小学生写法
		float distanceThreshold = 50; // 距离阈值
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
		    std::chrono::microseconds(1),
		    std::bind(&TargetSender::send_target, this));
	}

private:
	void send_target() {
		if(tracker_.target.aim_mode != 0) {
			message.aim_mode = tracker_.target.aim_mode;
			message.pitch_angle = tracker_.target.pitch_angle;
			message.yaw_angle = tracker_.target.yaw_angle;
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
			tracker_.rarmor.pos =
			    cv::Vec3d(armor.pos.x, armor.pos.y, armor.pos.z);
			tracker_.rarmor.ori =
			    cv::Vec3d(armor.ori.x, armor.ori.y, armor.ori.z);
			tracker_.rarmor.classes = armor.classes;
			tracker_.armors.push_back(tracker_.rarmor);
			std::cout << armor.pos.x << "|" << armor.pos.y << "|" << armor.pos.z
			          << std::endl;
		}
	}

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