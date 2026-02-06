#ifndef MAP_GLOBAL_H
#define MAP_GLOBAL_H
#include "common_functions.h"
#include <iostream>
#include "data_io.h"
#include <opencv2/opencv.hpp>

class MapGlobal 
{
public:
    MapGlobal(const MapParas_& map_paras);
    ~MapGlobal();


    void update(const cv::Mat& local_map);

    int getWidth() const;
    int getHeight() const;

    // const std::vector<unsigned char>& getMap() const;
    const cv::Mat& getMap() const { return map_;}
    cv::Mat& getMap() { return map_; } // 返回非 const 引用
    


private:
    float grid_size_;
    // unsigned char* map_;
    // std::vector<unsigned char> map_;
    cv::Mat map_;

    float max_x_;
    float max_y_;
    int width_;
    int height_;


};

#endif // MAP_GLOBAL_H
