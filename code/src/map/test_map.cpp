#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include "opencv2/opencv.hpp"
#include <filesystem>
#include "map_system.h"
#include "data_io.h"
#include <future>

namespace fs = std::filesystem;

struct tmpData {
    double x;
    double y;
    double yaw;
    int index;
};

std::vector<cv::Point2f> world_pts;
double x_min = -0.5;
double x_max = -0.3;
double y_min = -5.0;
double y_max =  5.0;

double step  =  0.05;   // 举例
// double x_min1 = -2.0;
// double x_max1 = 1.5;
// double y_min1 =  -0.1;
// double y_max1 =  0.1;


int main(int argc, char *argv[]) {

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <output_txt_path>" << std::endl;
        return 1; // Exit with error code
    }
    int choose = 0;
    if(argc == 3)
    {
        choose = std::stoi(argv[2]);
    }


    // std::random_device rd;
    // std::mt19937 gen(rd());  // 随机数引擎全局/外部保存，避免每次重新播种
    for (double x = x_min; x <= x_max ; x += step) {
        for (double y = y_min; y <= y_max ; y += step) {
            world_pts.push_back({x, y});
        }
    }

    // for (double x = x_min1; x <= x_max1 ; x += step) {
    //     for (double y = y_min1; y <= y_max1 ; y += step) {
    //         world_pts.push_back({x, y});
    //     }
    // }


    // generate_points()

    std::cout << "world_pts.size(): " << world_pts.size() << std::endl;
    // Assign the command-line arguments to variables
    std::string working_dir = argv[1]; // First argument: output file path

    std::string output_txt = working_dir + "/output.txt";
    std::string mask_dir = working_dir +  "/mask"; // mask文件夹路径

    // 1. 读取output.txt，存储到vector
    std::vector<tmpData> data_vec;
    std::ifstream infile(output_txt);
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string timestamp;
        int infer_index;
        double x, y, yaw;
        if (!(iss >> timestamp >> infer_index >> x >> y >> yaw)) continue;
        tmpData data = {x, y, yaw, infer_index};
        data_vec.push_back(data);
    }
    infile.close();

    // 输出总共有多少条数据
    std::cout << "Total data count: " << data_vec.size() << std::endl;

    // 2. 遍历mask文件夹，保存mask_index_xxx文件
    std::map<int, std::string> mask_map; // index -> mask文件名
    for (const auto& entry : fs::directory_iterator(mask_dir)) {
        std::string filename = entry.path().filename().string();
        if (filename.find("mask_index_") == 0) {
            // 提取xxx
            std::string index_str = filename.substr(std::string("mask_index_").length());
            try {
                int index = std::stoi(index_str);
                mask_map[index] = filename;
            } catch (...) {
                continue;
            }
        }
    }

    int lower_bound = 0;
    int upper_bound = 500;

    // int lower_bound = 10426;
    // int upper_bound = 10440;

    std::vector<SensorData_> sensor_datas;
    int processed_count = 0;
    double min_x_val = std::numeric_limits<double>::max();
    double max_x_val = std::numeric_limits<double>::lowest();
    double min_y_val = std::numeric_limits<double>::max();
    double max_y_val = std::numeric_limits<double>::lowest();

    for (const auto& data : data_vec) {
        if (data.index < lower_bound || data.index > upper_bound) continue;
        auto it = mask_map.find(data.index);
        if (it != mask_map.end()) {
            if (data.x < min_x_val) min_x_val = data.x;
            if (data.x > max_x_val) max_x_val = data.x;
            if (data.y < min_y_val) min_y_val = data.y;
            if (data.y > max_y_val) max_y_val = data.y;
            std::cout << "index: " << data.index
                      << " x: " << data.x
                      << " y: " << data.y
                      << " yaw: " << data.yaw
                      << " mask_file: " << it->second << std::endl;
            std::string mask_path = mask_dir + "/" + it->second;
            cv::Mat mask_color = cv::imread(mask_path);
            if (mask_color.empty()) {
                std::cerr << "Failed to read mask image: " << mask_path << std::endl;
                continue;
            }
            cv::Mat gray_img;
            cv::cvtColor(mask_color, gray_img, cv::COLOR_BGR2GRAY);

            SensorData_ sdata;
            sdata.mask = gray_img;
            sdata.idx = data.index;
            sdata.img_data.fusepose.xyz.x = data.x;
            sdata.img_data.fusepose.xyz.y = data.y;
            sdata.img_data.fusepose.rpy.z = data.yaw;


            double xr  = data.x;   // 机器人世界坐标 x
            double yr  = data.y;   // 机器人世界坐标 y
            double yaw = data.yaw;   // 机器人朝向（弧度，0 表示朝世界 x 正方向）
            double R   = 1.0;   // 探测半径
        
            double cos_yaw = std::cos(yaw);
            double sin_yaw = std::sin(yaw);
            double R2 = R * R;

            std::vector<cv::Point3f> robot_pts;  // 转到机器人坐标系后的点


            for (const auto& p : world_pts) {
                double dx = p.x - xr;
                double dy = p.y - yr;
        
                // 1) 先判断是否在半径内
                double dist2 = dx * dx + dy * dy;
                if (dist2 > R2) continue;
        
                // 2) 世界坐标 -> 机器人坐标
                cv::Point3f pr;
                pr.x =  dx * cos_yaw + dy * sin_yaw;
                pr.y = -dx * sin_yaw + dy * cos_yaw;
                pr.z = 0;
        
                robot_pts.push_back(pr);
            }
            std::cout << "robot_pts.size(): " << robot_pts.size() << std::endl;
            sdata.depth_img.depth = robot_pts;



            sensor_datas.push_back(sdata);
            processed_count++;
        }
    }

    std::cout << "Processed data count in index [" << lower_bound << "," << upper_bound << "]: " << processed_count << std::endl;

    // 以下地图参数与处理流程保持原样
    float min_x = -10.0;
    float min_y = -10.0;
    float max_x = 10.0f;
    float max_y = 10.0f;
    int global_gridw, global_gridh;
    float gird_size = 0.05f;

    int local_gridw = 100;
    int local_gridh = 100;

    float fextend_area = 2.50f;

    min_x = std::floor(min_x) - fextend_area;
    min_y = std::floor(min_y) - fextend_area;
    max_x = std::ceil(max_x) + fextend_area;
    max_y = std::ceil(max_y) + fextend_area;

    global_gridh = int((max_x - min_x) / gird_size);
    global_gridw = int((max_y - min_y) / gird_size);

    MapParas_ map_paras;
    map_paras.max_x = max_x;
    map_paras.max_y = max_y;
    map_paras.global_gridh = global_gridh;
    map_paras.global_gridw = global_gridw;
    map_paras.grid_size = 0.05f;
    map_paras.local_gridh = local_gridh;
    map_paras.local_gridw = local_gridw;
    map_paras.save_path = working_dir + "/x86/";
    map_paras.save_interval = 1;
    double scale1 = 640.0 / 544.0;
    scale1 = 1.0;
    cv::Mat new_K = (cv::Mat_<float>(3, 3) <<
        293.225983, 0.0, 312.401428,
        0.0, 293.225983 * scale1, 273.596466 * scale1,
        0.0, 0.0, 1.0
    );

    cv::Mat T_v_cam = (cv::Mat_<float>(4, 4) <<
        -0.0113301435, -0.179078743,   0.983769476,   0.506020546,
        -0.999895573,  -0.0067983577, -0.0127533963,  0.0254323687,
        0.00897187926, -0.983811259, -0.178983018,   0.197880179,
        0.0,           0.0,           0.0,           1.0
    );
    map_paras.cam_paras.K = new_K;
    map_paras.cam_paras.T_w_camL = T_v_cam;
    std::shared_ptr<MapSystem> map_system_ = std::make_shared<MapSystem>(map_paras);

    Timera T_ALL;
    for (const auto& sensor_data : sensor_datas) {
        std::cout << "Processing index: " << sensor_data.idx << std::endl;
        const cv::Mat map_ptr = map_system_->getMapPointer();
        std::cout << "depht_num: " << sensor_data.depth_img.depth.size() << std::endl;

        Timera t_all;  // 统计总共花费时间

        Timera t1;
        Timera t2;
    
    // int choose = 1;
    if(choose == 0)
    {
        map_system_->Input(sensor_data, map_ptr, true);
        // t1.out("input1 cost ");

        map_system_->Input1(sensor_data, map_ptr, true);
        // t2.out("input1 cost ");
        // t_all.out("all cost ");
    }

    // 创建两个线程，分别执行 Input 和 Input1

    if(choose == 1)
    {

        std::thread th_input([&](int id_choose) {
            std::cout << "id_choose: " << id_choose << std::endl;
            map_system_->Input(sensor_data, map_ptr, true);
            // t1.out("Input cost ");
        }, choose);

        std::thread th_input1([&]() {
            map_system_->Input1(sensor_data, map_ptr, true);
            // t2.out("Input1 cost ");
        });

        // 等待两个线程都执行完
        th_input.join();
        th_input1.join();
        // t_all.out("all cost ");
    }
    if(choose == 2)
    {

        // 有返回值
        // auto fa = std::async(std::launch::async, [&](){
        //     return map_system_->Input(sensor_data, map_ptr, true);
        
        // });

        auto f1 = std::async(std::launch::async, [&]() {
            map_system_->Input(sensor_data, map_ptr, true);
            // t1.out("Input cost ");
        });

        auto f2 = std::async(std::launch::async, [&]() {
            map_system_->Input1(sensor_data, map_ptr, true);
            // t2.out("Input1 cost ");
        });

        f1.get();
        f2.get();
    }

    }



    // print map size: 
    std::cout << "Processed data count in index [" << lower_bound << "," << upper_bound
    << "]: " << processed_count << std::endl;

    if (processed_count > 0) {
        std::cout << "X range: [" << min_x_val << ", " << max_x_val << "]" << std::endl;
        std::cout << "Y range: [" << min_y_val << ", " << max_y_val << "]" << std::endl;
    } else {
        std::cout << "No valid data in the given index range, X/Y range unavailable." << std::endl;
    }


    T_ALL.out("all process time cost: ");
    return 0;
}
