

#ifndef LIDAR_AI_DATA_H
#define LIDAR_AI_DATA_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
// #include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <random>

struct LiDARFrame {
    long long time_stamp = 0;
    cv::Point3f fusepose;  // x, y, z
    cv::Point3f rpy;       // roll, pitch, yaw
};

// AI 数据结构
struct AIFrame {
    long long time_stamp = 0;
    std::vector<cv::Point3f> xyz;  // 障碍物坐标
};

// 匹配后的数据结构
struct MatchedFrame {
    long long lidar_time_stamp;
    long long ai_time_stamp;
    cv::Point3f fusepose;
    cv::Point3f rpy;
    std::vector<cv::Point3f> xyz;
    long long diff;
    size_t pcl_no;
};


std::vector<AIFrame> loadAIData(const std::string& path);
std::vector<LiDARFrame> loadLiDARData(const std::string& path);
std::vector<MatchedFrame> matchingPoseWithAI(const std::vector<AIFrame>& ai_data, const std::vector<LiDARFrame>& li_data);
void findMinMaxXY(const std::vector<MatchedFrame>& frames, 
                  float& min_x, float& max_x, 
                  float& min_y, float& max_y);


std::vector<MatchedFrame> readAndParseData(const std::string& file_name);


std::vector<cv::Point> findValidContours(const cv::Mat& mask_img);
// std::vector<cv::Point> findValidContours(const cv::Mat& mask_img);
std::vector<std::vector<cv::Point>> findValidContours1(const cv::Mat& mask_img, int mini_area = 100, bool bSave = false);
std::vector<std::vector<cv::Point>> findValidContours2(const cv::Mat& mask_img, 
    int map_index, 
    bool bSave = false,
    int mini_area = 100
    );



std::vector<cv::Point> interpolate_polygon(const std::vector<cv::Point>& polygon, int max_distance = 10);
std::vector<std::vector<cv::Point>> interpolate_polygon1(const std::vector<std::vector<cv::Point>>& polygon, int max_distance = 10);

// get valid points
bool isBelowLine(int y, int b);


cv::Point lineIntersection(const cv::Point& p1, const cv::Point& p2, int b);

std::vector<cv::Point> interpolatePoints(const std::vector<cv::Point>& points, int maxDistance);

std::vector<cv::Point> getLowerPart(const std::vector<cv::Point>& polygon, int lineB = 348, int step = 2, int maxDistance = 5.0); // 338

cv::Point3f uv_to_world(float u, float v, const cv::Mat& K, const cv::Mat& T_w_cam);

#endif // LIDAR_AI_DATA_H
