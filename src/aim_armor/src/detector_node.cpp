#include <sys/types.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <interfaces/msg/armor.hpp>
#include <interfaces/msg/armors.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <interfaces/msg/aim_mode.hpp>

#include "lights.hpp"
#include "armor_detector.hpp"

using AimModemsg = interfaces::msg::AimMode;
using ImageMsg = sensor_msgs::msg::Image;
using ArmorsMsg = interfaces::msg::Armors;
using ArmorMsg = interfaces::msg::Armor;


class Detector {
public:
	// 从下位机接收的击打模式
	struct RunMode {
		// TODO: 默认应该是什么值
		ArmorColor color = RED;
		u_int8_t mode;
	} runmode;

public:
	Detector(){
		ad.init();
	};
	~Detector(){};

	cv::Mat frame;
	std::vector<Armor> armors;

	std::vector<Armor> get_armors() {
		armors.clear();
		if(!frame.empty()) {
			ad.match_armors(armors, frame, runmode.color);
		}
		return armors;
	}

private:
	ArmorDetector ad;
};


class Detector_Sender_Node: public rclcpp::Node {
public:
	Detector_Sender_Node(Detector &detector):
	Node("detector_sender_node"), detector_(detector) {
		subscription_ = this->create_subscription<ImageMsg>(
		    "camera/stream", 10,
		    std::bind(&Detector_Sender_Node::detector, this,
		              std::placeholders::_1));
		armors_publisher_ =
		    this->create_publisher<ArmorsMsg>("/detector/armors", 10);
	}

private:
	void detector(const ImageMsg::SharedPtr msg) {
		armors_msg.armors.clear();
		try {
			cv_ptr =
			    cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			detector_.frame = cv_ptr->image;
		} catch(cv_bridge::Exception &e) {
			RCLCPP_ERROR(this->get_logger(),
			             "Could not convert from '%s' to 'bgr8'.",
			             msg->encoding.c_str());
			return;
		}
		for(auto armor: detector_.get_armors()) {
			armor_msg.classes = armor.classes;
			armor_msg.pos.x = armor.pos[0];
			armor_msg.pos.y = armor.pos[1];
			armor_msg.pos.z = armor.pos[2];
			armor_msg.ori.x = armor.ori[0];
			armor_msg.ori.y = armor.ori[1];
			armor_msg.ori.z = armor.ori[2];
			armors_msg.armors.push_back(armor_msg);
		}
		armors_msg.header.frame_id = "1";
		armors_msg.header.stamp = this->get_clock()->now();
		armors_publisher_->publish(armors_msg);
	}

	Detector detector_;
	ArmorMsg armor_msg;
	ArmorsMsg armors_msg;
	cv_bridge::CvImagePtr cv_ptr;
	rclcpp::Subscription<ImageMsg>::SharedPtr subscription_;
	rclcpp::Publisher<ArmorsMsg>::SharedPtr armors_publisher_;
};


class Get_Mode_Node: public rclcpp::Node {
public:
	Get_Mode_Node(Detector &detector):
	Node("serial_sender_node"), detector_(detector) {
		// 订阅目标消息
		subscription = this->create_subscription<AimModemsg>(
		    "/serial/mode", 10,
		    std::bind(&Get_Mode_Node::setmode, this, std::placeholders::_1));
	}

private:
	void setmode(const AimModemsg::SharedPtr msg) {
		detector_.runmode.mode = msg->mode;
		if(msg->color == 'r') {
			detector_.runmode.color = RED;
		} else if(msg->color == 'b') {
			detector_.runmode.color = BLUE;
		} else {
			detector_.runmode.mode = RED;
		}
	}

	Detector &detector_;
	rclcpp::Subscription<AimModemsg>::SharedPtr subscription;
	rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char **argv) {
	Detector detector;
	rclcpp::init(argc, argv);

	auto detector_node = std::make_shared<Detector_Sender_Node>(detector);
	auto get_mode_node = std::make_shared<Get_Mode_Node>(detector);

	// 使用多线程执行器来管理节点
	rclcpp::executors::MultiThreadedExecutor executor;
	// 添加节点到执行器
	executor.add_node(detector_node);
	executor.add_node(get_mode_node);
	// 启动执行器
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
