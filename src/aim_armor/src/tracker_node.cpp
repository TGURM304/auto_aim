#include <cstdint>

#include <interfaces/msg/detail/target__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <interfaces/msg/armor.hpp>
#include <interfaces/msg/armors.hpp>


using ArmorMsg = interfaces::msg::Armor;
using TargetMsg = interfaces::msg::Target;


class Tracker {
public:
	Tracker(){
        target.aim_mode = 0;
        target.pitch_angle = .0;
        target.yaw_angle = .0;
    };

	~Tracker(){};

    // 状态
	enum State { SEARCHING, TRACKING };
	// 装甲板类别
	enum Classes { NUM1, NUM2, NUM3, NUM4, BASE, QSZ, SB, NONE};

    // 打击目标结构体（发送的数据）
	struct Target {
		uint8_t aim_mode;
		float pitch_angle;
		float yaw_angle;
	}target;

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

int main(){

}