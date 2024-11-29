#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <chrono>
#include <filesystem>

namespace fs = std::filesystem;

int main() {
	try {
		// 初始化 OpenVINO Core 对象
		ov::Core core;

		// 设置模型路径
		std::string model_path = "./assets/model/best-8.onnx";

		// 读取 ONNX 模型并转换为 OpenVINO 格式
		auto model = core.read_model(model_path);
		if(model == nullptr) {
			std::cerr << "加载模型失败" << std::endl;
			return -1;
		}

		// 获取模型的输入和输出名称
		auto input_name = model->input().get_any_name();
		auto output_name = model->output().get_any_name();

		// 类别标签
		std::vector<std::string> class_labels = {
		    "1",    "2",    "3",   "4", "5",
		    "base", "null", "qsz", "sb"};

		// 设置数据集根目录
		std::string dataset_dir = "./testsets";
		std::vector<std::string> data_list;
		std::vector<std::string> labels;

		// 遍历数据集并读取图片路径和标签
		for(const auto& label: class_labels) {
			std::string class_folder =
			    dataset_dir + "/" + label;
			if(fs::is_directory(class_folder)) {
				for(const auto& entry:
				    fs::directory_iterator(class_folder)) {
					if(entry.path().extension() == ".jpg"
					   || entry.path().extension()
					       == ".jpeg"
					   || entry.path().extension()
					       == ".png") {
						data_list.push_back(
						    entry.path().string());
						labels.push_back(label);
					}
				}
			}
		}

		// 初始化预测结果存储
		std::vector<std::string> predicted_labels;
		std::map<std::string, int>
		    correct_predictions_per_class;
		std::map<std::string, int> total_samples_per_class;

		// 创建 OpenVINO 推理会话
		auto compiled_model =
		    core.compile_model(model, "CPU");
		ov::InferRequest infer_request =
		    compiled_model.create_infer_request();

		// 遍历每一张图片进行推理
		for(size_t i = 0; i < data_list.size(); ++i) {
			auto start_time =
			    std::chrono::high_resolution_clock::now();

			// 加载图片
			cv::Mat image = cv::imread(data_list[i]);
			if(image.empty()) {
				std::cerr
				    << "无法读取图像: " << data_list[i]
				    << std::endl;
				continue;
			}

			// 图像预处理，调整为 64x64
			cv::resize(image, image, cv::Size(64, 64));
			image.convertTo(image, CV_32F,
			                1.0 / 255.0); // 归一化

			// 进行 (C, H, W) 维度转换 (RGB)
			cv::Mat channels[3];
			cv::split(
			    image,
			    channels); // 分割成三个通道（R, G, B）
			cv::Mat channels_merged[3] = {
			    channels[2], channels[1],
			    channels[0]}; // BGR -> RGB
			cv::Mat transposed_image;
			cv::merge(channels_merged, 3, transposed_image);

			// 转换为 OpenVINO 输入的 Tensor 格式 (1, C, H, W)
			ov::Shape input_shape = {
			    1, 64, 64,
			    3}; // 1个样本，3个通道，64x64图像
			ov::Tensor input_tensor(
			    ov::element::f32, input_shape,
			    transposed_image.data); // 使用 f32 类型
			infer_request.set_input_tensor(
			    input_tensor); // 设置输入张量

			// 进行推理
			infer_request.infer();

			// 获取推理结果
			auto output_tensor =
			    infer_request.get_output_tensor();
			auto output_data = output_tensor.data<float>();
			size_t num_classes =
			    output_tensor.get_shape()[1];

			// 获取预测类别（最大概率对应的类别）
			int predicted_class_idx = std::distance(
			    output_data,
			    std::max_element(
			        output_data,
			        output_data + num_classes));
			std::string predicted_class =
			    class_labels[predicted_class_idx];
			predicted_labels.push_back(predicted_class);

			// 更新每个类别的正确预测数量和总样本数量
			if(predicted_class == labels[i]) {
				correct_predictions_per_class[labels[i]] +=
				    1;
			}
			total_samples_per_class[labels[i]] += 1;

			// 记录推理时间
			auto end_time =
			    std::chrono::high_resolution_clock::now();
			std::chrono::duration<float> prediction_time =
			    end_time - start_time;
			std::cout << "Image: " << data_list[i]
			          << ", Predicted: " << predicted_class
			          << ", Actual: " << labels[i]
			          << ", Prediction Time: "
			          << prediction_time.count() * 1000
			          << " ms" << std::endl;
		}

		// 计算总准确率
		int correct_predictions = std::count_if(
		    predicted_labels.begin(),
		    predicted_labels.end(),
		    [&](const std::string& label) {
			    return label
			        == labels[&label
			                  - &predicted_labels[0]];
		    });
		float accuracy =
		    static_cast<float>(correct_predictions)
		    / predicted_labels.size() * 100;
		std::cout << "Model Accuracy: " << accuracy << "%"
		          << std::endl;

		// 计算每个类别的准确率
		for(const auto& label: class_labels) {
			if(total_samples_per_class[label] > 0) {
				float class_accuracy =
				    (static_cast<float>(
				         correct_predictions_per_class
				             [label])
				     / total_samples_per_class[label])
				    * 100;
				std::cout << "Accuracy for class " << label
				          << ": " << class_accuracy << "%"
				          << std::endl;
			} else {
				std::cout << "Accuracy for class " << label
				          << ": No samples in dataset"
				          << std::endl;
			}
		}

	} catch(const std::exception& ex) {
		std::cerr << "发生错误: " << ex.what() << std::endl;
		return -1;
	}

	return 0;
}
