#ifndef ROBOT_OBTS_H_
#define ROBOT_OBTS_H_
#include <array>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>



struct FrameData {
    cv::Point3f fusepose_xyz;     
    cv::Point3f fusepose_rpy;   
    // cv::point cam_local_postion;
    std::vector<cv::Point> car_pose;

    std::vector<int> ai_obts_len;
    std::vector<std::vector<int>> unique_index;
    std::vector<std::vector<cv::Point3f>> ai_obts; 
    std::vector<std::vector<cv::Point3f>> li_obts;      
    std::vector<std::vector<cv::Point3f>> world_pose_obts;       
    std::vector<std::vector<cv::Point>> world_grid_obts;       
    std::vector<std::vector<cv::Point>> local_grid_obts;       
    std::vector<std::vector<cv::Point3f>> pre_world_pose_obts;
    std::vector<std::vector<cv::Point>> pre_world_grid_obts;
    std::vector<std::vector<cv::Point>> pre_local_grid_obts;



    // 在 FrameData 中实现清空自身数据的功能
    void clear() {
        // 将 std::array 重置为默认值
        fusepose_xyz = {0.0f, 0.0f, 0.0f};
        fusepose_rpy = {0.0f, 0.0f, 0.0f};

        // ai_obts_len = 0;
        ai_obts_len.clear();

        // 清空 std::vector
        car_pose.clear();
        ai_obts.clear();
        li_obts.clear();
        unique_index.clear();
        world_pose_obts.clear();
        world_grid_obts.clear();
        local_grid_obts.clear();

        pre_world_pose_obts.clear();
        pre_world_grid_obts.clear();
        pre_local_grid_obts.clear();
    }
};

// debug here:
// 定义一个结构体来存储 LiDAR 数据
// struct LidarFrame {
//     long long time_stamp;
//     std::vector<float> fusepose;
//     std::vector<float> rpy;
//     std::vector<std::vector<float>> points;
// };

// // 定义一个结构体来存储 AI 数据
// struct AIFrame {
//     long long  time_stamp;
//     std::vector<std::vector<float>> xyz;
// };

#endif