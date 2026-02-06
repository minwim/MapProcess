#ifndef MAP_LOCAL_H
#define MAP_LOCAL_H

#include "map_global.h"
#include "robot_obts.h"
#include <opencv2/opencv.hpp>
#include "common_functions.h"
#include "lidar_ai_data.h"
#include <iterator>  // std::begin, std::end
#include <set>
#include <mutex>

#include "data_frame.h"
#include "data_lidar.h"
#include "data_io.h"
#include "utils.h"

using MapLocalCallback = std::function<void(const LocalMap_& local_data)>;

class MapLocal 
{
public:
    MapLocal(const MapParas_& map_paras_, cv::Mat global_map );
    ~MapLocal();


    // 注册回调函数
    void registerCallback(MapLocalCallback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback_ = std::move(callback);
    }
    // update src data
    void Input(const SensorData_& sensor_data, const cv::Mat& global_map, bool save);
    void Input1(const SensorData_& sensor_data, const cv::Mat& global_map, bool save);

    void updateFromGlobalMap(const MapGlobal& global_map, int robot_x, int robot_y);
    void update(const cv::Mat& map_ptr, 
        std::vector<std::vector<cv::Point3f>> pts,
        std::vector<std::vector<cv::Point3f>> inner_pts,
        const cv::Point3f& fusepose_xyz, const cv::Point3f& fusepose_rpy, 
        int ai_len, int inner_len,
        int i, bool save
    );

    void update1(const cv::Mat& map_ptr,
        std::vector<cv::Point3f> pts,
        const cv::Point3f& fusepose_xyz, const cv::Point3f& fusepose_rpy, 
        int i,
        bool save
    );

    const cv::Mat& getUpdatedLocalMapData() const ;


private:
    void printMap() const;

    // global map
    void getPriorMap(const std::vector<unsigned char>& map_ptr, int width, int height, const cv::Point3f& fusepose_xyz, cv::Point3f& fusepose_rpy);


    // draw visual
    void drawCar(cv::Mat& image, const std::vector<cv::Point>& car_pose, const std::string& output_name, int map_index_);
    void drawGCar(cv::Mat& image, const std::vector<cv::Point>& car_pose, const std::string& output_name, int map_index_);
    // int float2int(float prob);
    // float int2float(int vlaue);
private:
    int width_, height_, size_;
    int gwidth_;
    int gheight_;
    float max_x_;
    float max_y_;
    float grid_size_;
    // unsigned char* map_cur_;
    // unsigned char* map_pre_;

    // cur grid xy
    int cur_gx_;
    int cur_gy_;

    cv::Mat global_map_;
    cv::Mat track_map_;
    cv::Mat map_cur_;
    cv::Mat map_out_;
    cv::Mat map_pre_;
    FrameData frame_data_pre_;
    FrameData frame_data_cur_;
    FrameData frame_data_tmp_;


    //Lidar data handle
    cv::Mat global_map1_;
    cv::Mat track_map1_;
    cv::Mat map_cur1_;
    cv::Mat map_out1_;
    cv::Mat map_pre1_;
    FrameData frame_data_pre1_;
    FrameData frame_data_cur1_;
    FrameData frame_data_tmp1_;

    const std::vector<unsigned char> map_ref_;
    // std::vector<cv::Point2d> 

    int save_cnt_ = 0;


private:
    float MAP_SIZE = 2.0; // 2M
    float GRID_SIZE = 0.05; // every grid mean 0.05m
    int LOCAL_CENTER = 50; 

    float POSITIVE_ONE = 0.7;
    float POSITIVE_TWO = 0.6;

    float NEGATIVE_MISS = 0.4;
    
    std::string SAVE_PATH = "";
    int SAVE_INTERVAL = 0;

private:
    cv::Mat K;
    cv::Mat T_w_cam;

    float base_line = 59.998f;
    float bfx = 17593.093f;

    // 回调函数
    MapLocalCallback callback_;
    std::mutex callback_mutex_;  // 保护回调的线程安

};

#endif // MAP_LOCAL_H
