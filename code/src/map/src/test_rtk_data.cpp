#include "lidar_ai_data.h"
#include <iostream>

int main() {
    std::string li_path = "/home/ai/wlm/gitee/map_learing/04_map_grid_cxx/CXX/data/output1.txt";



    auto res = readAndParseData(li_path);

    std::cout << "LiDAR Data Loaded:\n";

    std::cout << "match.len: " << res.size() << std::endl;

}
