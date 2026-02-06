#ifndef COMMONFUNCTIONS_H
#define COMMONFUNCTIONS_H

#include <set>
// #include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc.hpp>
#include <unordered_set>

#include <cmath>
#include <vector>
#include <limits>
#include <iostream>
#include <functional>

// 自定义哈希函数，用于支持 unordered_set 存储 cv::Point
struct PointHash {
    std::size_t operator()(const cv::Point& point) const {
        return std::hash<int>()(point.x) ^ (std::hash<int>()(point.y) << 1);
    }
};

// 自定义比较函数，用于支持 unordered_set 存储 cv::Point
struct PointEqual {
    bool operator()(const cv::Point& lhs, const cv::Point& rhs) const {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
};


// Function to rotate a point around a given center
cv::Point2f rotatePoint(const cv::Point2f& point, const cv::Point2f& center, float angle);


// Function to rotate a point using a rotation matrix and a translation offset
cv::Point2f rotateAndTranslatePoint(const cv::Point2f& point, const cv::Mat& rotation_matrix, const cv::Point2f& offset);


std::vector<int> get_unique_indexes(const std::vector<cv::Point>& points);


bool is_in_2m_range(const cv::Point2f& center, const cv::Point2f& pt, int map_len = 40);


cv::Point world2grids(const cv::Point3f& pt, float gird_size = 0.05f);

// 返回两个vector<int>，分别是list1和list2中相同点的下标
std::pair<std::vector<int>, std::vector<int>> find_common_point_indices(const std::vector<cv::Point>& list1, const std::vector<cv::Point>& list2);


std::vector<cv::Point> select_points_by_indices(const std::vector<cv::Point>& points, const std::vector<int>& indices);


bool isValueInVector(const std::vector<int>& vec, int value);

unsigned char updateProbability(unsigned char P_O, float P_Z_given_O = 0.4f, float P_Z_given_not_O = 0.6f);


int pointToIndex(const cv::Point& pt, int width = 250, int height = 250);



bool calculateAngle(const cv::Point& p1, const cv::Point& p2,float yaw = 0.0f, int vis_r = 20, int vis_deg = 65);


std::vector<cv::Point> getPolygonInnerPoints(const std::vector<cv::Point>& frame_data_cur, const std::vector<int>& unique_indexes);

std::vector<cv::Point> getPolygonInnerPoints1(const std::set<cv::Point>& frame_data_cur);

#endif // COMMONFUNCTIONS_H
