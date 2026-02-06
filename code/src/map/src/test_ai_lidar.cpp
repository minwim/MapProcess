#include "lidar_ai_data.h"
#include <iostream>

int main() {
    std::string li_path = "/home/JSDC/017254/code/gitee/map_learing/03_map_grid_python/04_xyzrpy_visual/rpy/data4/li.txt";
    std::string ai_path = "/home/JSDC/017254/code/gitee/map_learing/03_map_grid_python/04_xyzrpy_visual/rpy/data4/ai.txt";

    // 7424
    auto lidar_frames = loadLiDARData(li_path);
    // 7042
    auto ai_frames = loadAIData(ai_path);

    auto res = matchingPoseWithAI(ai_frames, lidar_frames);

    std::cout << "LiDAR Data Loaded:\n";
    std::cout << "li.len: " << lidar_frames.size() << std::endl;
    std::cout << "ai.len: " << ai_frames.size() << std::endl;
    std::cout << "match.len: " << res.size() << std::endl;

}
