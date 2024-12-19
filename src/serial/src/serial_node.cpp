#include <cstdint>
#include <interfaces/msg/detail/aim_mode__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <interfaces/msg/target.hpp>
#include <interfaces/msg/aim_mode.hpp>
#include <chrono>
#include "serial.hpp"

using TargrtMsg = interfaces::msg::Target;
using AimModeMsg = interfaces::msg::AimMode;

class SerialReceiver : public rclcpp::Node {
public:
    SerialReceiver(Serial &serial): Node("serial_receiver_node"), serial_(serial) {
        // 创建一个发布者，发布 AimModeMsg 类型的消息
        publisher = this->create_publisher<AimModeMsg>("/serial/mode", 10);

        // 设置定时器，每1秒发布一次消息
        timer = this->create_wall_timer(
            std::chrono::microseconds(1),
            std::bind(&SerialReceiver::timer_callback, this));
    }

private:
    void timer_callback() {
        // 创建消息并发布
        auto message = AimModeMsg();
        message.color = 1;
        message.mode = 0;
        
        // 假设你要在这里调用 serial.sendData()
        Serial::Data data;  // 定义适当的数据
        serial_.sendData(data);  // 调用 sendData
        
        publisher->publish(message);
    }

    Serial &serial_;  // 引用 Serial 类
    rclcpp::Publisher<AimModeMsg>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};


class SerialSender : public rclcpp::Node {
public:
    SerialSender(Serial &serial): Node("serial_sender_node"), serial_(serial) {
        // 订阅目标消息
        subscription = this->create_subscription<TargrtMsg>(
            "/target", 10, std::bind(&SerialSender::sender, this, std::placeholders::_1));
    }

private:
    void sender(const TargrtMsg::SharedPtr msg) {
        Serial::Data data;
        data.mode = msg->aim_mode;
        data.pitch_angle = msg->pitch_angle;
        data.yaw_angle = msg->yaw_angle;
        serial_.sendData(data);
    }

    Serial &serial_;
    rclcpp::Subscription<TargrtMsg>::SharedPtr subscription;
	rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char **argv) {
	// 初始化串口
    Serial serial;
    serial.init();

    rclcpp::init(argc, argv);

    // 创建节点，并传入 Serial 实例
    auto publisher_node = std::make_shared<SerialReceiver>(serial);
    auto subscriber_node = std::make_shared<SerialSender>(serial);

    // 使用多线程执行器来管理节点
    rclcpp::executors::MultiThreadedExecutor executor;
    // 添加节点到执行器
    executor.add_node(publisher_node);
    executor.add_node(subscriber_node);
    // 启动执行器
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
