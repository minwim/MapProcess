#ifndef DATA_FRAME_H
#define DATA_FRAME_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <set>
#include "common_functions.h"
struct PointComparator {
    bool operator()(const cv::Point& p1, const cv::Point& p2) const {
        if (p1.x != p2.x) {
            return p1.x < p2.x;
        }
        return p1.y < p2.y;
    }
};
// cv::Point 的哈希函数
struct PtHash {
    std::size_t operator()(const cv::Point& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

// cv::Point 的相等比较
struct PtEqual {
    bool operator()(const cv::Point& a, const cv::Point& b) const {
        return a.x == b.x && a.y == b.y;
    }
};

class OrderedUniquePoints {
public:
    bool insert(const cv::Point& p) {
        if (set_.find(p) == set_.end()) {
            set_.insert(p);
            vec_.push_back(p);
            return true;
        }
        return false;
    }

    const std::vector<cv::Point>& data() const { return vec_; }

private:
    std::vector<cv::Point> vec_;
    std::unordered_set<cv::Point, PtHash, PointEqual> set_;
};



#define DEBUG_FRAME 1
class FramesData {
public:
    // 构造函数
    FramesData(cv::Point3f fusepose_xyz, cv::Point3f fusepose_rpy, 
               const std::vector<std::vector<cv::Point3f>>& ai_obts,
               const std::vector<std::vector<cv::Point3f>>& inner_obts,
               float max_x, float max_y,
               int ai_len = 0, int inner_len = 0, int map_index = 0);

    // 初始化函数
    void Init();

    // 坐标转换相关函数
    void transformCoords(const cv::Point3f& local_points, int contour_index);
    void transformPoints(const std::vector<cv::Point3f>& pts, int contour_index);

    // 帧数据处理相关函数
    void calcCamLocalPose();
    void handleCureFramesData();


public:
    std::vector<cv::Point> getPolygonInnerPoints1(const std::set<cv::Point, PointComparator>& frame_data_cur);
    // std::vector<std::vector<cv::Point>> setToVector(const std::vector<std::set<cv::Point, PtHash>>& set_data);
    std::vector<std::vector<cv::Point>> setToVector(const std::vector<OrderedUniquePoints>& set_data);

    // 获取帧地图
    cv::Mat getFrameMap() const { return perceptionMap_; }
    std::vector<cv::Point> getCarLocalPose() const { return car_pose_; };
    std::vector<cv::Point> getCarGlocalPose() const { return car_gpose_; };
    cv::Point getGlobalxy() const {return gxy_;};

private:
    // 成员变量
    cv::Point3f fusepose_xyz_;
    cv::Point3f fusepose_rpy_;
    std::vector<std::vector<cv::Point3f>> ai_obts_;
    std::vector<std::vector<cv::Point3f>> inner_obts_;
    float max_x_;
    float max_y_;
    int ai_len_;
    int inner_len_;

    std::vector<cv::Point> car_pose_;
    std::vector<cv::Point> car_gpose_;
    std::vector<std::vector<cv::Point3f>> world_pose_obts_;

    // std::vector<std::set<cv::Point, PointComparator>> world_grid_obts_; 
    // std::vector<std::set<cv::Point, PointComparator>> local_grid_obts_;  

    std::vector<OrderedUniquePoints> world_grid_obts_; 
    std::vector<OrderedUniquePoints> local_grid_obts_;

    cv::Point gxy_;

    int frame_width_;
    int frame_height_;
    cv::Mat perceptionMap_;
    int map_index_;
    // 栅格相关参数
    float GRID_SIZE = 0.05;
    int LOCAL_CENTER = 50;

    // 工具函数

};

#endif // DATA_FRAME_H
