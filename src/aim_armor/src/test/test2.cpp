#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <iostream>
#include <algorithm>  // std::max_element

int main() {
    try {
        // 初始化 OpenVINO Core 对象
        ov::Core core;

        // 加载 ONNX 模型
        auto model = core.read_model("./assets/model/best-8.onnx");
        auto compiled_model = core.compile_model(model, "CPU");
        auto infer_request = compiled_model.create_infer_request();

        // 加载图像并预处理
        cv::Mat image = cv::imread("testsets/base/2024_11_27_5ixts.jpg");
        cv::resize(image, image, cv::Size(64, 64));
        image.convertTo(image, CV_32F, 1.0 / 255.0);  // 归一化

        // 转换图像格式为 Tensor
        ov::Shape input_shape = {1, 64, 64, 3};  // NHWC
        ov::Tensor input_tensor1(ov::element::f32, input_shape, image.data);
        infer_request.set_input_tensor(input_tensor1);

        // 推理并获取结果
        infer_request.infer();
        auto output_tensor = infer_request.get_output_tensor();
        auto output_data = output_tensor.data<float>();

        // 获取最大概率对应的类别
        int predicted_class = std::distance(output_data, std::max_element(output_data, output_data + output_tensor.get_size()));
        std::cout << "Predicted class: " << predicted_class << std::endl;

    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return -1;
    }

    return 0;
}
