#include "map_system.h"
#include "robot.h"
#include "lidar_ai_data.h"
#include <chrono> // 用于时间测量
#include <cmath> // 包含 floor 和 ceil 函数
#include <thread>   // 用于 sleep_for 和 sleep_until


int main() 
{
    // 01-数据准备
    // std::string li_path = "/home/ai/wlm/gitee/map_learing/01_binocular_vision_xuce/output1_mono.txt";
    // std::string li_path = "/home/ai/wlm/gitee/map_learing/04_map_grid_cxx/CXX/data/output2_pcl.txt";
    // std::string li_path = "/home/ai/wlm/gitee/map_learing/01_binocular_vision_xuce/output1_mono_change.txt";
    // std::string li_path = "/home/ai/wlm/gitee/map_learing/01_binocular_vision_xuce/output1_mono_area.txt";
    // std::string li_path = "./output1_mono_area.txt";
    // std::string li_path = "/home/JSDC/017254/code/gitee/map_learing/01_binocular_vision_xuce/output1_mono_area.txt";

    // std::string li_path = "/home/ai/wlm/gitee/cxx_-area_-occupy_-map/data/output1_mono_area.txt";
    std::string li_path = "/home/ai/wlm/gitee/map_learing/01_binocular_vision_xuce/txts/0709_outputs_mono_vision.txt";
    auto start1 = std::chrono::high_resolution_clock::now();

    auto res = readAndParseData(li_path);

    std::cout << "match.len: " << res.size() << std::endl;
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count();
    // std::vector<MatchedFrame> res = matchingPoseWithAI(ai_frames, lidar_frames);

    // 02-计算地图的长宽
    float min_x, max_x, min_y, max_y;
    findMinMaxXY(res, min_x, max_x, min_y, max_y);
    std::cout << "Min X: " << min_x << ", Max X: " << max_x << std::endl;
    std::cout << "Min Y: " << min_y << ", Max Y: " << max_y << std::endl;
    float fextend_area = 2.50f;
    min_x = std::floor(min_x);
    min_y = std::floor(min_y);
    max_x = std::ceil(max_x) + fextend_area;
    max_y = std::ceil(max_y) + fextend_area;
    std::cout << "adjust\n";
    std::cout << "Min X: " << min_x << ", Max X: " << max_x << std::endl;
    std::cout << "Min Y: " << min_y << ", Max Y: " << max_y << std::endl;
    int global_gridw, global_gridh;
    float gird_size = 0.05f;
    global_gridw = int((max_x - min_x) / gird_size) + int(fextend_area / 0.05f);
    global_gridh = int((max_y - min_y) / gird_size) + int(fextend_area / 0.05f);
    std::cout << "global_gridw: " << global_gridw << std::endl;
    std::cout << "global_gridh: " << global_gridh << std::endl;

    // 04-全局地图 局部地图构建
    int local_gridw = 100;
    int local_gridh = 100;

    MapSystem map_system(max_x, max_y, global_gridw, global_gridh, local_gridw, local_gridh);

    // 05-刷新地图，打印地图
    // 记录起始时间
    auto start = std::chrono::high_resolution_clock::now();
    // 输出耗时
    // int len = 1000;
    int len = res.size();
    // for(int i = 100 ; i < 800; i++)
    // int len = 800;
    // int len = res.size();
    for(int i = 100 ; i < len; i = i + 2)
    {
        MatchedFrame tmp = res[i];
        std::vector<cv::Point3f> pts = tmp.xyz;
        cv::Point3f fuse_xyz = tmp.fusepose;
        cv::Point3f fuse_rpy = tmp.rpy;

        // 获取指针
        const cv::Mat map_ptr = map_system.getMapPointer();

        // 获取局部
        map_system.updateLocalMap(map_ptr, pts, fuse_xyz, fuse_rpy, i);

        cv::Mat local_map = map_system.getUpdatedLocalMapData();

        // 更新全局
        // map_system.updateGlobalMap(local_map);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));

    }


    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "read 耗时: " << duration1 << " 毫秒" << std::endl;
    std::cout << "耗时: " << duration << " 毫秒" << std::endl;




    return 0;
}
