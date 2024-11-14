import os
import cv2
import numpy as np
from openvino.runtime import Core
from sklearn.metrics import accuracy_score
import time  # 用于测量预测时长
from collections import defaultdict

def load_model(model_xml, model_bin, device="CPU"):
    """加载并编译OpenVINO模型"""
    ie = Core()
    model = ie.read_model(model=model_xml, weights=model_bin)
    compiled_model = ie.compile_model(model=model, device_name=device)
    return compiled_model

def load_data(dataset_dir, class_labels):
    """加载数据集中的图片路径和对应标签"""
    data_list = []
    labels = []
    for label in class_labels:
        class_folder = os.path.join(dataset_dir, label)
        if os.path.isdir(class_folder):
            for img_name in os.listdir(class_folder):
                img_path = os.path.join(class_folder, img_name)
                if img_path.endswith(('.jpg', '.jpeg', '.png')):  # 确保是图片文件
                    data_list.append(img_path)
                    labels.append(label)
    return data_list, labels

def preprocess_image(img_path, input_size=(64, 64)):
    """加载和预处理图片"""
    image = cv2.imread(img_path)
    image = cv2.resize(image, input_size)
    image = image.transpose(2, 0, 1)  # 改为 (C, H, W)
    image = np.expand_dims(image, axis=0)  # 添加批次维度 (1, C, H, W)
    image = np.float32(image) / 255.0  # 归一化到 [0, 1]
    return image

def evaluate_model(compiled_model, data_list, labels, class_labels):
    """进行推理并计算准确率"""
    predicted_labels = []
    correct_predictions_per_class = defaultdict(int)
    total_samples_per_class = defaultdict(int)
    
    for i, img_path in enumerate(data_list):
        start_time = time.time()  # 记录开始时间
        
        # 预处理图片
        image = preprocess_image(img_path)
        
        # 进行推理
        result = compiled_model([image])[compiled_model.output(0)]
        output_scores = result[0]
        
        # 获取得分最高的类别索引
        predicted_class_idx = np.argmax(output_scores)
        predicted_class = class_labels[predicted_class_idx]
        predicted_labels.append(predicted_class)

        # 更新每个类别的正确预测数量和总样本数量
        if predicted_class == labels[i]:
            correct_predictions_per_class[labels[i]] += 1
        total_samples_per_class[labels[i]] += 1

        # 记录结束时间并计算时长
        end_time = time.time()
        prediction_time = end_time - start_time

        # 输出图片预测结果及时长
        print(
            f"Image: {img_path}, Predicted: {predicted_class}, Actual: {labels[i]}, Prediction Time: {prediction_time*1000:.4f} ms")
    
    return predicted_labels, correct_predictions_per_class, total_samples_per_class

def calculate_accuracy(labels, predicted_labels, class_labels, correct_predictions_per_class, total_samples_per_class):
    """计算总准确率和每个类别的准确率"""
    accuracy = accuracy_score(labels, predicted_labels)
    print(f"Model Accuracy: {accuracy * 100:.2f}%")

    for label in class_labels:
        if total_samples_per_class[label] > 0:
            class_accuracy = (correct_predictions_per_class[label] / total_samples_per_class[label]) * 100
            print(f"Accuracy for class {label}: {class_accuracy:.2f}%")
        else:
            print(f"Accuracy for class {label}: No samples in dataset")

def main():
    # 1. 定义模型路径和设备
    model_xml = "./model/openvino_model/best.xml"
    model_bin = "./model/openvino_model/best.bin"
    dataset_dir = 'testsets'
    class_labels = ['1', '2', '3', '4', '5', 'null', 'sb']

    # 2. 加载模型
    compiled_model = load_model(model_xml, model_bin)

    # 3. 加载数据集
    data_list, labels = load_data(dataset_dir, class_labels)

    # 4. 进行推理并评估模型
    predicted_labels, correct_predictions_per_class, total_samples_per_class = evaluate_model(compiled_model, data_list, labels, class_labels)

    # 5. 计算并输出准确率
    calculate_accuracy(labels, predicted_labels, class_labels, correct_predictions_per_class, total_samples_per_class)

if __name__ == "__main__":
    main()
