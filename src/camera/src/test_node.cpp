// #include <cstdio>

// #include <opencv2/opencv.hpp>

#include "mindvision.hpp"


// int main() {
// 	auto mv = MindVision();
// 	mv.init(2);

// 	while(1) {
// 		cv::Mat frame = mv.getFrame();
// 		if(!frame.empty()) {
// 			// 显示图像
// 			cv::imshow("Camera Frame", frame);
// 			cv::waitKey(1);
// 		} else {
// 			printf("Failed to capture image.\n");
// 			break;
// 		}
// 	}
// 	return 0;
// }


int main() {
	MindVision camera_;
	camera_.init(2);
	camera_.record("/home/interweave", 60000);
	return 0;
}