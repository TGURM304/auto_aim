#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <interfaces/msg/armor.hpp>
#include <interfaces/msg/armors.hpp>
#include <interfaces/msg/target.hpp>


using ArmorsMsg = interfaces::msg::Armors;
using ArmorMsg = interfaces::msg::Armor;
using TargetMsg = interfaces::msg::Target;


class Tracker {
public:
	Tracker() {
		target.aim_mode = 0;
		target.pitch_angle = .0;
		target.yaw_angle = .0;
	};

	~Tracker(){};

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

	// 接收到的装甲板结构体
	struct Armor {
		std::string number;
		float distance;
	} armor;


	bool isFindArmor() {
		// TODO: 是否找到装甲板

		return true;
	}

	bool isSameBoard() {
		// TODO: 判断是否是同一装甲板
		return true;
	}

	Target tracking() {
		switch(currentState) {
		case SEARCHING: {
			break;
		}
		case TRACKING: {
			break;
		}
		default: {
			currentState = SEARCHING;
		}
		}
		return target;
	}

private:
	// 当前跟踪状态
	State currentState = SEARCHING;
	Classes previousClass = NONE;
};

class TargetSender: public rclcpp::Node {
public:
	TargetSender(Tracker &tracker):
	Node("serial_receiver_node"), tracker_(tracker) {
		// 创建一个发布者，发布 AimModeMsg 类型的消息
		publisher = this->create_publisher<TargetMsg>("/target/armor", 10);

		// 设置定时器，每1秒发布一次消息
		timer = this->create_wall_timer(
		    std::chrono::microseconds(1),
		    std::bind(&TargetSender::timer_callback, this));
	}

private:
	void timer_callback() {
		// 创建消息并发布
		auto message = TargetMsg();

		publisher->publish(message);
	}

	Tracker tracker_;
	rclcpp::Publisher<TargetMsg>::SharedPtr publisher;
	rclcpp::TimerBase::SharedPtr timer;
};


class ArmorsReceiver: public rclcpp::Node {
public:
	ArmorsReceiver(Tracker &tracker): Node("detector_node"), tracker_(tracker) {
		// 订阅目标消息
		subscription = this->create_subscription<ArmorsMsg>(
		    "/detector/armors", 10,
		    std::bind(&ArmorsReceiver::sender, this, std::placeholders::_1));
	}

private:
	void sender(const ArmorsMsg::SharedPtr armors_msg){
		for(auto armor :armors_msg->armors){
			tracker_.armor.distance = armor.distance_to_image_center;
			tracker_.armor.number = armor.number;
		}
	}

	Tracker tracker_;
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