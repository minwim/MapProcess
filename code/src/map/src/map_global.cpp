#include "map_global.h"

#include <algorithm>

MapGlobal::MapGlobal(const MapParas_& map_paras)
    :max_x_(map_paras.max_x), max_y_(map_paras.max_y), 
    width_(map_paras.global_gridw), height_(map_paras.global_gridh) , grid_size_(map_paras.grid_size)
{
    // map_ = new unsigned char[size_];
    // std::fill(map_, map_ + size_, 128);
    // 手动初始化 map_
    // map_.resize(size_);               // 调整 vector 大小
    // std::fill(map_.begin(), map_.end(), 128); // 填充所有元素为 128

    // map_ = cv::Mat::ones(height_, width_, CV_8UC1) * 128;     
    map_ = cv::Mat::ones(height_, width_, CV_8UC1) * 128;     

    std::cout << "hw: " << height_ << " " << width_ << std::endl;
    std::cout << "save map_" << std::endl;
    cv::imwrite("ori_map.png", map_);
}

MapGlobal::~MapGlobal() 
{
    // delete[] map_;
}


int MapGlobal::getWidth() const
{
    return width_;
}

int MapGlobal::getHeight() const 
{
    return height_;
}



void MapGlobal::update(const cv::Mat& local_map)
{
    // TO DO
    std::cout << "update Global Map " << std::endl;
    std::cout << "map_: " << std::endl;

}




