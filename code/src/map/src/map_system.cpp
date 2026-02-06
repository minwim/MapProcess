#include "map_system.h"

MapSystem::MapSystem(const MapParas_& map_paras)
    : global_map_(map_paras), 
    local_map_(map_paras, global_map_.getMap()) 
{
    std::cout << "Map system build"<<std::endl;
}

void MapSystem::updateLocalMap(const cv::Mat& map_ptr, std::vector<std::vector<cv::Point3f>> pts,
    std::vector<std::vector<cv::Point3f>> inner_pts,
     const cv::Point3f& fuse_xyz, const cv::Point3f& fuse_rpy,
     int ai_len, int inner_len,
     int i, bool save)
{
    // local_map_.update2(map_ptr, pts, fuse_xyz, fuse_rpy, i);
    local_map_.update(map_ptr, pts, inner_pts, fuse_xyz, fuse_rpy, ai_len, inner_len, i, save);
    
}

void MapSystem::Input(const SensorData_& sensor_data, const cv::Mat& global_map, bool save)
{
    local_map_.Input(sensor_data, global_map, save);
}

void MapSystem::Input1(const SensorData_& sensor_data, const cv::Mat& global_map, bool save)
{
    local_map_.Input1(sensor_data, global_map, save);
}

void MapSystem::updateGlobalMap(const cv::Mat& local_map)
{
    global_map_.update(local_map);
}



const cv::Mat& MapSystem::getUpdatedLocalMapData()
{
    return local_map_.getUpdatedLocalMapData();
}



const cv::Mat& MapSystem::getMapPointer() const
{
    return global_map_.getMap();
}
