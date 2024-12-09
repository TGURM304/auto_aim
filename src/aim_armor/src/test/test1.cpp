cv::Mat matrix; // 相机内参
cv::Mat dist; // 畸变参数

auto dist_toml = config["camera"]["dist"].as_array();
auto matrix_toml = config["camera"]["matrix"].as_array();

// 相机内参 matrix
matrix = cv::Mat(3, 3, CV_64F);
int matrix_idx = 0;
for (const auto& value : *matrix_toml) {
    matrix.at<double>(matrix_idx / 3, matrix_idx % 3) = value.value_or(0.0);
    matrix_idx++;
}

// 畸变系数 dist
dist = cv::Mat(1, 5, CV_64F);
int dist_idx = 0;
for (const auto& value : *dist_toml) {
    dist.at<double>(dist_idx) = value.value_or(0.0);
    dist_idx++;
}

// test
std::cout << "内参(matrix):" << std::endl;
std::cout << matrix << std::endl;
std::cout << "畸变(dist):" << std::endl;
std::cout << dist << std::endl;

auto start = std::chrono::high_resolution_clock::now();
cv::Mat frame_undistort;
cv::undistort(frame, frame_undistort, matrix, dist);
auto end = std::chrono::high_resolution_clock::now();
    
std::chrono::duration<double> duration = end - start;
std::cout << "time: " << duration.count() * 1000 << " ms" << std::endl;

