#ifndef DATA_IO_HPP
#define DATA_IO_HPP
#ifdef USE_DLOG
#include "dlog/dlog.h"
#endif
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <opencv2/opencv.hpp>

struct FusePose_ {
    int idx;
    uint64_t timestamp;
    cv::Point3f xyz;
    cv::Point3f rpy;

    FusePose_(int id, uint64_t ts, const cv::Point3f& position, const cv::Point3f& rotation)
        : idx(id), timestamp(ts), xyz(position), rpy(rotation) {}

    FusePose_() : FusePose_(0, 0, cv::Point3f(0.0f, 0.0f, 0.0f), cv::Point3f(0.0f, 0.0f, 0.0f)) {}
};

struct DepthData_ {
    int idx;
    int height;
    int width;
    uint64_t timestamp;
    // cv::Mat depth;
    std::vector<cv::Point3f> depth;
    FusePose_ fusepose;

    DepthData_(int id, int h, int w, uint64_t ts, /*const cv::Mat& dp,*/ const std::vector<cv::Point3f>& dp, const FusePose_& pose)
        : idx(id), height(h), width(w), timestamp(ts), depth(dp), fusepose(pose) {}

    DepthData_() : DepthData_(0, 0, 0, 0, std::vector<cv::Point3f>(), FusePose_()) {}
};

struct ImageData_ {
    int idx;
    int height;
    int width;
    float gain;
    float exposure;
    uint64_t timestamp;
    cv::Mat image;
    FusePose_ fusepose;

    ImageData_(int id, int h, int w, float ga, float ex, uint64_t ts,
               const cv::Mat& img, const FusePose_& pose)
        : idx(id), height(h), width(w), gain(ga), exposure(ex), timestamp(ts),
          image(img), fusepose(pose) {}

    ImageData_() : ImageData_(0, 0, 0, 0.0f, 0.0f, 0, cv::Mat(), FusePose_()) {}
};

struct SensorData_ {
    int idx;
    int idx1;
    uint64_t timestamp;
    DepthData_ depth_img;
    ImageData_ img_data;
    FusePose_ fuse_pose;
    cv::Mat mask;

    SensorData_(int id, int id1, uint64_t ts, const DepthData_& depth, const ImageData_& img,
                const FusePose_& pose, const cv::Mat& mk)
        : idx(id), idx1(id1), timestamp(ts), depth_img(depth), img_data(img),
          fuse_pose(pose), mask(mk) {}

    SensorData_()
        : SensorData_(0, 0, 0, DepthData_(), ImageData_(), FusePose_(), cv::Mat()) {}
};


struct LocalMap_ {
    uint64_t timestamp;
    int index;
    float x;
    float y;
    float z;
    int width;
    int height;
    cv::Mat local_map;
};




struct CamParas_ {
    float base_line;
    float bfx;
    cv::Mat K;
    cv::Mat T_w_camL;

    CamParas_()
    {
        K = cv::Mat::eye(3, 3, CV_32FC1);
        T_w_camL = cv::Mat::eye(4, 4, CV_32FC1);
    }
};

// Mapsystem
struct MapParas_ {
    float max_x;
    float max_y;
    float global_gridw; 
    float global_gridh;
    float local_gridw;
    float local_gridh;
    float grid_size = 0.05f;
    int save_interval = 0;


    std::string save_path;
    CamParas_ cam_paras;

    MapParas_(){
        save_path = "";
        CamParas_();
    }
};


// SaveImg
struct SaveTask_ {
    std::string path;
    cv::Mat image;
};



struct ReadCamParams_ {
    std::string camera_sn;
    int origin_width, origin_height;
    int width, height;
    float base_line; // mm
    float bxf;

    cv::Mat KD_cam;
    cv::Mat T_w_camL; // ex
    


    ReadCamParams_()
    {
        camera_sn = "";
        KD_cam = cv::Mat::eye(3, 3, CV_32FC1);
        T_w_camL = cv::Mat::eye(4, 4, CV_32FC1);
    }
    
};

#endif // DATA_IO_HPP
