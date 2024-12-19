#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "mindvision.hpp"
#include "hikvision.hpp"
#include "toml.hpp"

class StreamNode: public rclcpp::Node {
public:
    StreamNode() : Node("stream_node") {
        config = toml::parse_file("./assets/config.toml");
        camera_version = config["camera"]["version"].value_or("MV");
        while (true) {
            try {
                if (camera_version == "MV") {
                    camera_mv.init(2);
                    break;
                } else if (camera_version == "HK") {
                    nRet = camera_hk.init();
                    if(nRet != MV_OK){
                        RCLCPP_ERROR(this->get_logger(), "Error code: 0x%x", nRet);
                    }
                    break;
                } else {
                    std::printf("配置文件错误，无法识别的相机类型: %s，尝试重新获取配置\n", camera_version.c_str());
					config = toml::parse_file("./assets/config.toml");
                	camera_version = config["camera"]["version"].value_or("MV");
                }
            } catch (const std::exception& e) {
                std::printf("读取配置文件失败: %s\n", e.what());
            }
        }
		
        // 创建发布器和定时器
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/stream", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(0), std::bind(&StreamNode::publish, this));
    }

private:
    void publish() {
        if (camera_version == "MV") {
            frame = camera_mv.getFrame();
        } else if (camera_version == "HK") {
            result = camera_hk.getFrame();
            if(result.second == MV_OK){
                frame = result.first;
            }
            else{
                frame = result.first;
                RCLCPP_ERROR(this->get_logger(), "Error code: 0x%x", result.second);
            }
        } else {
            frame = camera_mv.getFrame();
        }
        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        }
        else{
            std::cout << "+++" << std::endl;
        }
    }

    toml::table config;
    std::string camera_version;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    MindVision camera_mv;
    HikVision camera_hk;
    cv::Mat frame;
    std::pair<cv::Mat, int> result;
    int nRet;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StreamNode>());
    rclcpp::shutdown();
    return 0;
}
