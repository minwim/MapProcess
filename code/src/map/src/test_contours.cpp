#include "lidar_ai_data.h"
#include <chrono> // 用于时间测量
#include <cmath> // 包含 floor 和 ceil 函数
#include <thread>   // 用于 sleep_for 和 sleep_until
#include <iostream>

int main()
{
    // 示例：相机内参矩阵 K
    cv::Mat K = (cv::Mat_<float>(3, 3) << 257.96665348,  0.       , 493.0518361,
    0.  ,       257.65757889 ,302.4293594,
    0.0, 0.0, 1.0);

    // 示例：相机到世界的变换矩阵 T_w_cam
    cv::Mat T_w_cam = (cv::Mat_<float>(4, 4) << 0.00274878, -0.141637, 0.989915, 0.466449,
    -0.999923, 0.0115661, 0.00443145, 0.0242619,
    -0.0120771, -0.989851, -0.141595, 0.221014, 
    0.0, 0.0, 0.0, 1.0);

    // Example usage: Load a sample RGB image
    std::string mask_path = "./1744185613103514.jpg";
    cv::Mat rgb_img = cv::imread(mask_path); // Replace with your image path
    if (rgb_img.empty()) {
        std::cerr << "Error: Could not load the image!" << std::endl;
        return -1;
    }

    auto start = std::chrono::high_resolution_clock::now(); // 总体开始时间

    for (size_t i = 0; i < 1000; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        // Step 1: Extract contours from the RGB image
        auto step1_start = std::chrono::high_resolution_clock::now();
        std::vector<cv::Point> contourPoints1 = findValidContours(rgb_img);
        auto step1_end = std::chrono::high_resolution_clock::now();
        auto step1_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step1_end - step1_start).count();
        std::cout << "Step 1 (findValidContours) 耗时: " << step1_duration << " 毫秒" << std::endl;

        // Step 2: Interpolate the polygon
        auto step2_start = std::chrono::high_resolution_clock::now();
        std::vector<cv::Point> contourPoints2 = interpolate_polygon(contourPoints1);
        auto step2_end = std::chrono::high_resolution_clock::now();
        auto step2_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step2_end - step2_start).count();
        std::cout << "Step 2 (interpolate_polygon) 耗时: " << step2_duration << " 毫秒" << std::endl;

        // Step 3: Get the lower part of the polygon
        auto step3_start = std::chrono::high_resolution_clock::now();
        std::vector<cv::Point> contourPoints = getLowerPart(contourPoints2);
        auto step3_end = std::chrono::high_resolution_clock::now();
        auto step3_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step3_end - step3_start).count();
        std::cout << "Step 3 (getLowerPart) 耗时: " << step3_duration << " 毫秒" << std::endl;

        std::cout << "pts.size: " << contourPoints.size() << std::endl;

        // Step 4: Convert UV to world coordinates
        auto step4_start = std::chrono::high_resolution_clock::now();
        std::vector<cv::Point3f> res;
        for (size_t idx = 0; idx < contourPoints.size(); idx++)
        {
            cv::Point3f xyz_world = uv_to_world(contourPoints[idx].x, contourPoints[idx].y, K, T_w_cam );
            res.push_back(xyz_world);
        }
        auto step4_end = std::chrono::high_resolution_clock::now();
        auto step4_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step4_end - step4_start).count();
        std::cout << "Step 4 (uv_to_world) 耗时: " << step4_duration << " 毫秒" << std::endl;

        if (!contourPoints.empty()) {
            // Draw the contour points on the original image
            for (const auto& point : contourPoints) {
                // cv::circle(rgb_img, point, 3, cv::Scalar(0, 0, 255), -1); // Draw a red circle at each point
            }

            // Display the result
            // cv::imshow("Contour Points on Original Image", rgb_img);
            // cv::waitKey(0);
        } else {
            std::cout << "No contours were extracted." << std::endl;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now(); // 总体结束时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "总耗时: " << duration << " 毫秒" << std::endl;

    return 0; 
}
