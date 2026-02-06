#ifndef DATA_LIDAR_H
#define DATA_LIDAR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <set>
#include "common_functions.h"



// 需要去重，也可以自定义比较规则，比如按 (x,y) 排序：
struct PointLess {
    bool operator()(const cv::Point& a, const cv::Point& b) const {
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x;
    }
};
#define DEBUG_FRAME 1
class FramesLidar {
public:
    // 构造函数
    FramesLidar(cv::Point3f fusepose_xyz, cv::Point3f fusepose_rpy, 
               const std::vector<cv::Point3f> pts,
               float max_x, float max_y,
               int map_index = 0);

    // 初始化函数
    void Init();

    // 坐标转换相关函数
    void transformCoords(const cv::Point3f& local_points, int contour_index);
    void transformPoints(const std::vector<cv::Point3f>& pts, int contour_index);

    // 帧数据处理相关函数
    void calcCamLocalPose();
    void handleCureFramesLidar();


public:

    // 获取帧地图
    cv::Mat getFrameMap() const { return perceptionMap_; }
    std::vector<cv::Point> getCarLocalPose() const { return car_pose_; };
    std::vector<cv::Point> getCarGlocalPose() const { return car_gpose_; };
    cv::Point getGlobalxy() const {return gxy_;};

private:
    // 成员变量
    cv::Point3f fusepose_xyz_;
    cv::Point3f fusepose_rpy_;
    float max_x_;
    float max_y_;

    std::vector<cv::Point> car_pose_;
    std::vector<cv::Point> car_gpose_;
    std::vector<cv::Point3f> world_pose_obts_;

    std::vector<cv::Point3f> pts_;

    std::set<cv::Point, PointLess> world_grid_obts_;
    std::set<cv::Point, PointLess> local_grid_obts_;


    // std::vector<std::set<cv::Point, PointComparator>> world_grid_obts_; 
    // std::vector<std::set<cv::Point, PointComparator>> local_grid_obts_;  


    cv::Point gxy_;

    int frame_width_;
    int frame_height_;
    cv::Mat perceptionMap_;
    int map_index_;
    // 栅格相关参数
    float GRID_SIZE = 0.05;
    int LOCAL_CENTER = 50;


};

#endif // DATA_LIDAR_H
