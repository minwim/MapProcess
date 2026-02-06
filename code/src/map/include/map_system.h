#ifndef MAP_SYSTEM_H
#define MAP_SYSTEM_H

#include "map_global.h"
#include "map_local.h"
#include <opencv2/core.hpp>

class MapSystem 
{
public:
    MapSystem(const MapParas_& map_paras);

    void updateLocalMap(const cv::Mat& map_ptr, std::vector<std::vector<cv::Point3f>> pts,
        std::vector<std::vector<cv::Point3f>> inner_pts,
         const cv::Point3f& fuse_xyz, const cv::Point3f& fuse_rpy,
         int ai_len, int inner_len,
         int i, bool save = true);

    const cv::Mat& getUpdatedLocalMapData();

    void updateGlobalMap(const cv::Mat& local_map);

    const cv::Mat& getMapPointer() const;

    void Input(const SensorData_& sensor_data, const cv::Mat& global_map, bool save);
    void Input1(const SensorData_& sensor_data, const cv::Mat& global_map, bool save);

    MapLocal& getLocalMap() {return local_map_;};

private:
    
    MapGlobal global_map_;
    MapLocal local_map_;
};

#endif // MAP_SYSTEM_H


// cv::Point3d(p.x, p.y, p.z)