#include "map_local.h"
#include <iostream>
#include <algorithm>

MapLocal::MapLocal(const MapParas_& map_paras, cv::Mat global_map)
    :max_x_(map_paras.max_x),max_y_(map_paras.max_y),
    gwidth_(map_paras.global_gridw), gheight_(map_paras.global_gridh),
    width_(map_paras.local_gridw), height_(map_paras.local_gridh), 
    size_(map_paras.local_gridh * map_paras.local_gridw), grid_size_(map_paras.grid_size)

{

    map_cur_ = cv::Mat::ones(height_, width_, CV_8UC1) * 128;  // 初始化为 128
    map_pre_ = cv::Mat::ones(height_, width_, CV_8UC1) * 128;     

    global_map_ = global_map;
    global_map1_ = global_map.clone();
    track_map_ = cv::Mat::ones(gwidth_, gheight_, CV_8UC1);
    track_map1_ = cv::Mat::ones(gwidth_, gheight_, CV_8UC1);
    LOCAL_CENTER = int(width_ / 2);

    SAVE_INTERVAL = map_paras.save_interval;
    SAVE_PATH = map_paras.save_path;

    cv::imwrite("global_map_in_local_class.png", global_map_);
    cv::imwrite("global_map1_in_local_class.png", global_map1_);

    float scale_x = 640.0 / 544.0;
    // K = (cv::Mat_<float>(3, 3) << 298.404816, 0., 315.682831,
    //                             0., 298.404816 * scale_x, 274.352478 * scale_x,
    //                             0.0, 0.0, 1.0);

    // 示例：相机到世界的变换矩阵 T_w_cam
    // T_w_cam = (cv::Mat_<float>(4, 4) << 0.00274878, -0.141637, 0.989915, 0.466449,
                                       // -0.999923, 0.0115661, 0.00443145, 0.0242619,
                            //            -0.0120771, -0.989851, -0.141595, 0.221014,
                            // 0.0, 0.0, 0.0, 1.0);

                            // [293.22598, 0, 312.62671;
                            //     0, 293.22598, 273.86121;
                            //     0, 0, 1]
                            //    [0.0017364123, -0.18423127, 0.98288137, 0.4676708;
                            //     -0.99999845, 6.02414e-06, 0.0017677814, 0.029750489;
                            //     -0.00033160162, -0.98288292, -0.18423097, 0.18760425;
                            //     0, 0, 0, 1]

    K = map_paras.cam_paras.K.clone();
    K.at<float>(1,1) *= scale_x;
    K.at<float>(1,2) *= scale_x;

    T_w_cam = map_paras.cam_paras.T_w_camL.clone();
    bfx = map_paras.cam_paras.bfx;
    base_line = map_paras.cam_paras.base_line;

}

MapLocal::~MapLocal() {

}


void MapLocal::getPriorMap(const std::vector<unsigned char>& map_ptr, int width, int height, const cv::Point3f& fusepose_xyz, cv::Point3f& fusepose_rpy)
{
    // 获取全局栅格坐标
    cv::Point fuse_grid = world2grids(fusepose_xyz);

}


void MapLocal::updateFromGlobalMap(const MapGlobal& global_map, int robot_x, int robot_y) 
{
    int half_width = width_ / 2;
    int half_height = height_ / 2;
}


const cv::Mat& MapLocal::getUpdatedLocalMapData() const 
{
    return  map_out_;
}

void MapLocal::Input1(const SensorData_& sensor_data, const cv::Mat& gmap, bool save)
{
    int idx = sensor_data.idx;
    cv::Mat out_mask = sensor_data.mask;
    FusePose_ fuse_pose = sensor_data.img_data.fusepose;
    DepthData_ depth_data = sensor_data.depth_img;
    auto pts = depth_data.depth;
    #ifdef USE_DLOG
    LOG_I("sensor_data: idx: {} idx1 {}", sensor_data.idx, sensor_data.idx1);
    LOG_I("img: idx: {} pose idx {}", sensor_data.img_data.idx, sensor_data.fuse_pose.idx);
    LOG_I("pose {} {}, yaw: {}", fuse_pose.xyz.x, fuse_pose.xyz.y, fuse_pose.rpy.z);
    #endif

    std::cout << "pts.size(): " << pts.size() << std::endl;
    update1(gmap, pts, fuse_pose.xyz, fuse_pose.rpy, idx, save);


}



void MapLocal::Input(const SensorData_& sensor_data, const cv::Mat& gmap, bool save)
{
    int idx = sensor_data.idx;
    cv::Mat out_mask = sensor_data.mask;
    FusePose_ fuse_pose = sensor_data.img_data.fusepose;
    DepthData_ depth_data = sensor_data.depth_img;
    #ifdef USE_DLOG
    LOG_I("sensor_data: idx: {} idx1 {}", sensor_data.idx, sensor_data.idx1);
    LOG_I("img: idx: {} pose idx {}", sensor_data.img_data.idx, sensor_data.fuse_pose.idx);
    LOG_I("pose {} {}, yaw: {}", fuse_pose.xyz.x, fuse_pose.xyz.y, fuse_pose.rpy.z);
    #endif

    // if(idx == 11)
    // {
    //     cv::Mat tstimg = sensor_data.mask;
    //     cv::imwrite("wlm.png", tstimg);
    // }

    std::cout << "input detph.size(): " << sensor_data.depth_img.depth.size() << std::endl;

#if 0
    if(idx == 10)
    {
            // 输出 PCD 文件名
        std::string pcd_filename = "point_cloud.pcd";
        std::vector<cv::Vec3f> point_cloud;  // 存储有效 3D 点（world_x, world_y, world_z）

        cv::Mat depth_img = depth_data.depth;


        // 遍历图像的每个像素
        for (int y = 0; y < depth_data.height; y++) {
            for (int x = 0; x < depth_data.width; x++) {
                uint16_t disparity = depth_img.at<uint16_t>(y, x);
                float cam_depth = (disparity != 0) ? (bfx * 256 ) / disparity : 0;
                float cam_x  = (x - cx) * cam_depth / fx;
                float cam_y  = (y - cx) * cam_depth / fy;

                cv::Mat P_cam_homo = (cv::Mat_<float>(4, 1) << cam_x, cam_y, cam_depth, 1.0f);
                cv::Mat P_world_homo = T_w_cam * P_cam_homo;

                float w = P_world_homo.at<float>(3, 0); 
                float world_x = P_world_homo.at<float>(0, 0) / w;
                float world_y = P_world_homo.at<float>(1, 0) / w;
                float world_z = P_world_homo.at<float>(2, 0) / w;
            

                point_cloud.emplace_back(world_x, world_y, world_z);

            }
        }

        std::ofstream pcd_file(pcd_filename);
        if (!pcd_file.is_open()) {
            std::cerr << "无法创建 PCD 文件！" << std::endl;
            // return 1;
        }
    
        // 写入 PCD 文件头
        int point_count = point_cloud.size();
        pcd_file << "# .PCD v0.7" << std::endl;
        pcd_file << "VERSION 0.7" << std::endl;
        pcd_file << "FIELDS x y z" << std::endl;  // 仅 3D 坐标
        pcd_file << "SIZE 4 4 4" << std::endl;    // 每个字段 4 字节（float）
        pcd_file << "TYPE F F F" << std::endl;    // 数据类型：float
        pcd_file << "COUNT 1 1 1" << std::endl;   // 每个字段的元素数
        pcd_file << "WIDTH " << point_count << std::endl;  // 点云总数
        pcd_file << "HEIGHT 1" << std::endl;      // 无序点云（1 表示非结构化）
        pcd_file << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;  // 视点（默认原点）
        pcd_file << "POINTS " << point_count << std::endl;
        pcd_file << "DATA ascii" << std::endl;    // ASCII 格式（易读）
    
        // 写入所有有效点的坐标
        for (const auto& point : point_cloud) {
            pcd_file << point[0] << " " << point[1] << " " << point[2] << std::endl;
        }
    
        pcd_file.close();
        std::cout << "PCD 文件保存成功！共 " << point_count << " 个有效点" << std::endl;
    }
    

#endif


    if (out_mask.empty())
    {
        #ifdef USE_DLOG
        LOG_I("Error1: Could not load the image!");
        #endif

        return;
    }

    #if 0
        cv::Mat inter_mask = out_mask.clone();
            // 将插值后的点绘制到 out_mask 图像上
        for (const auto& contour : contourPoints2) {
            for(const auto& point : contour)
            {
                cv::circle(inter_mask, point, 2, cv::Scalar(255, 255, 255), -1); // 绘制红色点，半径为 2

            }
        }
        cv::imwrite("inter_mask.png", inter_mask);

    #endif

    cv::Mat rgb_img;
    cv::cvtColor(out_mask, rgb_img, cv::COLOR_GRAY2BGR);

    // step 1: Get lower part

    int start_x = 5;
    int start_y = 348;
    int lower_h = start_y;
    int roi_w = rgb_img.cols - start_x - 5;
    int roi_h = rgb_img.rows - start_y - 5;
    cv::Rect roi(start_x, start_y, roi_w, roi_h);
    cv::Mat lower_img = rgb_img(roi);

    // Step 2: Extract contours from the RGB image
    // std::vector<cv::Point> contourPoints1 = contourPoints1(lower_img);
    // std::vector<std::vector<cv::Point>> contourPoints1 = findValidContours1(lower_img);
    std::vector<std::vector<cv::Point>> contourPoints1 = findValidContours2(lower_img, idx, true, 100);
    // std::cout << "contourPoints1.size() : " << contourPoints1.size() << std::endl;
    std::vector<cv::Point> tmp_all;
    tmp_all.push_back(cv::Point(0, 0));
    tmp_all.push_back(cv::Point(640, 0));
    tmp_all.push_back(cv::Point(640, 640 - start_y));
    tmp_all.push_back(cv::Point(0, 640 - start_y));
    contourPoints1.push_back(tmp_all);
    // Step 2: Interpolate the polygon
    // std::vector<cv::Point> contourPoints2 = interpolate_polygon(contourPoints1);
    std::vector<std::vector<cv::Point>> contourPoints2 = interpolate_polygon1(contourPoints1);
    std::cout << "contourPoints2.size() : " << contourPoints2.size() << std::endl;

#if 0
        cv::Mat inter_mask = out_mask.clone();
            // 将插值后的点绘制到 out_mask 图像上
        for (const auto& contour : contourPoints2) {
            for(const auto& point : contour)
            {
                cv::Point p1(point.x, point.y + lower_h);
                cv::circle(inter_mask, p1, 2, cv::Scalar(255, 255, 255), -1); // 绘制红色点，半径为 2

            }
        }
        std::string inter_name = SAVE_PATH +  "/perception/inter_" + std::to_string(idx) + ".png";

        cv::imwrite(inter_name, inter_mask);
#endif

    // Step 4: Convert UV to world coordinates
    std::vector<std::vector<cv::Point3f>> res;
    for (auto contour : contourPoints2)
    {
        std::vector<cv::Point3f> tmp;

        for (size_t idx = 0; idx < contour.size(); idx++)
        {
            // int h = 348;
            cv::Point3f xyz_world = uv_to_world(contour[idx].x, contour[idx].y + lower_h, K, T_w_cam);
            tmp.push_back(xyz_world);
        }
        res.push_back(tmp);
    }

        std::vector<std::vector<cv::Point3f>> pts = res;

        update(gmap, pts, pts, fuse_pose.xyz, fuse_pose.rpy, pts.size(), pts.size(), idx, save);


}

void MapLocal::update1(const cv::Mat& map_ptr,
    std::vector<cv::Point3f> pts,
    const cv::Point3f& fusepose_xyz, const cv::Point3f& fusepose_rpy, 
    int map_index,
    bool save
)
{
    std::cout << " ==========index: " << map_index << std::endl;
    FramesLidar framesLidar(fusepose_xyz, fusepose_rpy, pts, max_x_, max_y_, map_index);
    framesLidar.Init();
    framesLidar.handleCureFramesLidar();

    // 处理上一帧的数据在当前坐标下的表示
    int grid_cu = (max_y_ - fusepose_xyz.y) / GRID_SIZE;
    int grid_cv = (max_x_ - fusepose_xyz.x) / GRID_SIZE;
    int start_u = grid_cu - width_ / 2;
    int start_v = grid_cv - height_ / 2;
    #ifdef USE_DLOG
    LOG_I("start uv {} {}", start_u, start_v);
    #endif
    

    cv::Rect roi(start_u, start_v , width_, height_);
    std::cout << "startuvwh: " << start_u << " " << start_v << " " << width_ << " " << height_ << std::endl;
    std::cout << "global: " << global_map_.rows << global_map_.cols << std::endl;

    cv::Mat map_pre =  global_map1_(roi);
    // update
    cv::Mat map_preception = framesLidar.getFrameMap();
    cv::Mat map_cur = cv::Mat::ones(height_, width_, CV_8UC1) * 128;  // 初始化为 128


    int cnt_255 = 0;
    int cnt_0 = 0;

    // 引用概率更新
    for (size_t x = 0; x < map_preception.rows; x++)
    {
        for (size_t y = 0; y < map_preception.cols; y++)
        {
            int grid_value = map_pre.at<unsigned char>(y,x);
            float grid_prob;
            // 0:obts  128:unkown  255:grass
            if(map_preception.at<unsigned char>(y,x) == 0) 
            {
                if(grid_value < 40)
                {
                    continue;
                }
                else
                {
                    grid_prob = 0.4 * (grid_value / 255.0);
                    map_pre.at<unsigned char> (y,x)= updateProbability(grid_value, grid_prob, 1 - grid_prob);
                }
            }
            else if(map_preception.at<unsigned char>(y,x)== 255) 
            {
                if(grid_value > 215)
                {
                    continue;
                }
                else
                {
                    grid_prob = 0.4 * (grid_value / 255.0) + 0.6;
                    map_pre.at<unsigned char> (y,x)= updateProbability(grid_value, grid_prob, 1 - grid_prob);
                }
            }
            else // unkown
            {
                ;
            }
        }
        
    }

    #ifdef USE_DLOG
    LOG_I("0-255 {} {}", cnt_0, cnt_255);
    #endif

    // copy to global 
    // un-need
    auto gxy = framesLidar.getGlobalxy();

    //track
    track_map_.at<unsigned char>(gxy.y, gxy.x) = 0;
    cv::Mat outputImg = map_pre.clone();

    int lower =  int(255 * 0.15); // 64 = 
    int higher = int(255 * 0.85); // 191 = 
    outputImg.setTo(255, map_pre >= higher);
    outputImg.setTo(0, map_pre <= lower);
    outputImg.setTo(128, (map_pre > lower) & (map_pre < higher));
    cv::Mat map_out;
    cv::cvtColor(outputImg, map_out, cv::COLOR_GRAY2BGR);
    map_out1_ = outputImg.clone();  // 如果 map_out_ 为空，直接赋值   


    if(SAVE_INTERVAL != 0)
{
    if (save_cnt_ % SAVE_INTERVAL == 0)
    {
        std::cout << "save_cnt: " << save_cnt_ << std::endl;
        std::string global_name = SAVE_PATH +  "/global/global1_" + std::to_string(map_index) + ".png";
        std::string track_name = SAVE_PATH +  "/track/track1_" + std::to_string(map_index) + ".png";
        std::string local_name = SAVE_PATH + "/local/local1_" + std::to_string(map_index) + ".png";
        std::string perception_name = SAVE_PATH + "/perception/perception1_" + std::to_string(map_index) + ".png";
        std::string output_name = SAVE_PATH + "/output/output1_" + std::to_string(map_index) + ".png";

        std::vector<cv::Point> car_pose = framesLidar.getCarLocalPose();
        std::vector<cv::Point> car_gpose = framesLidar.getCarGlocalPose();
        // draw car
        cv::Mat  carImg = map_preception.clone();
        drawCar(carImg, car_pose, perception_name, map_index);

        // saveMatAsBin(output_name, map_out_);
        cv::imwrite(output_name, map_out1_ );
        cv::Mat localImg = cv::Mat::ones(height_, width_, CV_8UC3) * 128;  // Initialize all channels to 128
        // 遍历数组，将值大于 128 的点绘制到图像中
        for (int x = 0; x < map_pre.rows; x++)  
        {      
            for (int y = 0; y < map_pre.cols; y++) {   
                int value = map_pre.at<unsigned char>(x,y);
                localImg.at<cv::Vec3b>(x,y) = cv::Vec3b(value, value, value);

            }
        }

        drawCar(localImg, car_pose, local_name, map_index);

        cv::Mat mask = (track_map_ == 0);
        #if 0
            cv::Mat globalImg = global_map_.clone();
            globalImg.setTo(0, mask);
        #endif

        #if 1
            cv::Mat globalImg = global_map1_.clone();
            cv::Mat trackImg = cv::Mat::zeros(global_map_.size(), global_map_.type());
            trackImg.setTo(255,  mask);
            cv::imwrite(track_name, trackImg);
            // saveMatAsBin(track_name, trackImg);

        #endif
        drawGCar(globalImg, car_gpose, global_name, map_index);



    }
}

}



void MapLocal::update(const cv::Mat& map_ptr, 
    std::vector<std::vector<cv::Point3f>> pts,
    std::vector<std::vector<cv::Point3f>> inner_pts,
    const cv::Point3f& fusepose_xyz, const cv::Point3f& fusepose_rpy, 
    int ai_len, int inner_len,
    int map_index,
    bool save)
{
    // 处理本帧数据
    FramesData framesData(fusepose_xyz, fusepose_rpy, pts, inner_pts, max_x_, max_y_,  ai_len, inner_len, map_index);
    framesData.Init();
    framesData.handleCureFramesData();

    // 处理上一帧的数据在当前坐标下的表示
    int grid_cu = (max_y_ - fusepose_xyz.y) / GRID_SIZE;
    int grid_cv = (max_x_ - fusepose_xyz.x) / GRID_SIZE;
    int start_u = grid_cu - width_ / 2;
    int start_v = grid_cv - height_ / 2;
    #ifdef USE_DLOG
    LOG_I("start uv {} {}", start_u, start_v);
    #endif
    cv::Rect roi(start_u, start_v , width_, height_);
    std::cout << "startuvwh: " << start_u << " " << start_v << " " << width_ << " " << height_ << std::endl;
    std::cout << "global: " << global_map_.rows << global_map_.cols << std::endl;

    cv::Mat map_pre =  global_map_(roi);
    // update
    cv::Mat map_preception = framesData.getFrameMap();
    cv::Mat map_cur = cv::Mat::ones(height_, width_, CV_8UC1) * 128;  // 初始化为 128

    int cnt_255 = 0;
    int cnt_0 = 0;

    // 引用概率更新
    for (size_t x = 0; x < map_preception.rows; x++)
    {
        for (size_t y = 0; y < map_preception.cols; y++)
        {
            int grid_value = map_pre.at<unsigned char>(y,x);
            float grid_prob;
            // 0:obts  128:unkown  255:grass
            if(map_preception.at<unsigned char>(y,x) == 0) 
            {
                if(grid_value < 40)
                {
                    continue;
                }
                else
                {
                    grid_prob = 0.4 * (grid_value / 255.0);
                    map_pre.at<unsigned char> (y,x)= updateProbability(grid_value, grid_prob, 1 - grid_prob);
                }
            }
            else if(map_preception.at<unsigned char>(y,x)== 255) 
            {
                if(grid_value > 215)
                {
                    continue;
                }
                else
                {
                    grid_prob = 0.4 * (grid_value / 255.0) + 0.6;
                    map_pre.at<unsigned char> (y,x)= updateProbability(grid_value, grid_prob, 1 - grid_prob);
                }
            }
            else // unkown
            {
                ;
            }
        }
        
    }

    #ifdef USE_DLOG
    LOG_I("0-255 {} {}", cnt_0, cnt_255);
    #endif

    // copy to global 
    // un-need
    auto gxy = framesData.getGlobalxy();

    //track
    track_map_.at<unsigned char>(gxy.y, gxy.x) = 0;
    cv::Mat outputImg = map_pre.clone();

    int lower =  int(255 * 0.15); // 64 = 
    int higher = int(255 * 0.85); // 191 = 
    outputImg.setTo(255, map_pre >= higher);
    outputImg.setTo(0, map_pre <= lower);
    outputImg.setTo(128, (map_pre > lower) & (map_pre < higher));
    cv::Mat map_out;
    cv::cvtColor(outputImg, map_out, cv::COLOR_GRAY2BGR);
    map_out_ = outputImg.clone();  // 如果 map_out_ 为空，直接赋值


if(SAVE_INTERVAL != 0)
{
    if (save_cnt_ % SAVE_INTERVAL == 0)
    {
        std::cout << "save_cnt: " << save_cnt_ << std::endl;
        std::string global_name = SAVE_PATH +  "/global/global_" + std::to_string(map_index) + ".png";
        std::string track_name = SAVE_PATH +  "/track/track_" + std::to_string(map_index) + ".png";
        std::string local_name = SAVE_PATH + "/local/local_" + std::to_string(map_index) + ".png";
        std::string perception_name = SAVE_PATH + "/perception/perception_" + std::to_string(map_index) + ".png";
        std::string output_name = SAVE_PATH + "/output/output_" + std::to_string(map_index) + ".png";

        std::vector<cv::Point> car_pose = framesData.getCarLocalPose();
        std::vector<cv::Point> car_gpose = framesData.getCarGlocalPose();
        // draw car
        cv::Mat  carImg = map_preception.clone();
        drawCar(carImg, car_pose, perception_name, map_index);

        // saveMatAsBin(output_name, map_out_);
        cv::imwrite(output_name, map_out_ );
        cv::Mat localImg = cv::Mat::ones(height_, width_, CV_8UC3) * 128;  // Initialize all channels to 128
        // 遍历数组，将值大于 128 的点绘制到图像中
        for (int x = 0; x < map_pre.rows; x++)  
        {      
            for (int y = 0; y < map_pre.cols; y++) {   
                int value = map_pre.at<unsigned char>(x,y);
                localImg.at<cv::Vec3b>(x,y) = cv::Vec3b(value, value, value);

            }
        }

        drawCar(localImg, car_pose, local_name, map_index);

        cv::Mat mask = (track_map_ == 0);
        #if 0
            cv::Mat globalImg = global_map_.clone();
            globalImg.setTo(0, mask);
        #endif

        #if 1
            cv::Mat globalImg = global_map_.clone();
            cv::Mat trackImg = cv::Mat::zeros(global_map_.size(), global_map_.type());
            trackImg.setTo(255,  mask);
            cv::imwrite(track_name, trackImg);
            // saveMatAsBin(track_name, trackImg);

        #endif
        drawGCar(globalImg, car_gpose, global_name, map_index);



    }
}

    #ifdef USE_DLOG
    LOG_I("prepare send local map: ");
    #endif
    save_cnt_++;
    {
        LocalMap_ local_data;
        local_data.timestamp = 0;  
        local_data.index     =   0;
        local_data.x         = fusepose_xyz.x + 50 * grid_size_;
        local_data.y         = fusepose_xyz.y + 50 * grid_size_;
        local_data.z         = fusepose_rpy.z;
        local_data.width     = map_out_.cols;
        local_data.height    = map_out_.rows;
        local_data.local_map = map_out_.clone();
        #ifdef USE_DLOG
        // LOG_I("prepare send local map1: ");
        #endif
        std::lock_guard<std::mutex> lock(callback_mutex_);
        #ifdef USE_DLOG
        // LOG_I("prepare send local map2: ");
        #endif
        if (callback_) {
            callback_(local_data);
        } else {
        }

    }

}



void MapLocal::printMap() const 
{
}





void MapLocal::drawCar(cv::Mat& image, const std::vector<cv::Point>& car_pose, const std::string& output_name, int map_index) 
{
    // 确保 car_pose 中至少包含 6 个点
    if (car_pose.size() < 6) {
        throw std::invalid_argument("car_pose must contain at least 6 points.");
    }
    auto p1 = cv::Point(car_pose[1].x, car_pose[1].y); 
    auto p2 = cv::Point(car_pose[2].x, car_pose[2].y);
    auto p3 = cv::Point(car_pose[3].x, car_pose[3].y);
    auto p4 = cv::Point(car_pose[4].x, car_pose[4].y);
    auto c1 = cv::Point(car_pose[5].x, car_pose[5].y);

    cv::line(image, p1, p2, cv::Scalar(255), 1);
    cv::line(image, p2, p3, cv::Scalar(255), 1);
    cv::line(image, p3, p4, cv::Scalar(255), 1);
    cv::line(image, p4, p1, cv::Scalar(255), 1);


    cv::putText(image, std::to_string(map_index), cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);


    cv::circle(image, c1, 1, cv::Scalar(255), 1);


    // 保存图片
    cv::imwrite(output_name, image);
    // saveMatAsBin(output_name, image);
}


void MapLocal::drawGCar(cv::Mat& image, const std::vector<cv::Point>& car_pose, const std::string& output_name, int map_index) 
{
    // 确保 car_pose 中至少包含 6 个点
    if (car_pose.size() < 6) {
        throw std::invalid_argument("car_pose must contain at least 6 points.");
    }
    auto p1 = cv::Point(car_pose[1].x, car_pose[1].y); // 调换 x 和 y
    auto p2 = cv::Point(car_pose[2].x, car_pose[2].y);
    auto p3 = cv::Point(car_pose[3].x, car_pose[3].y);
    auto p4 = cv::Point(car_pose[4].x, car_pose[4].y);
    auto c1 = cv::Point(car_pose[5].x, car_pose[5].y);

    cv::line(image, p1, p2, cv::Scalar(0), 1);
    cv::line(image, p2, p3, cv::Scalar(0), 1);
    cv::line(image, p3, p4, cv::Scalar(0), 1);
    cv::line(image, p4, p1, cv::Scalar(0), 1);


    cv::putText(image, std::to_string(map_index), cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0), 1);

    cv::circle(image, c1, 1, cv::Scalar(0), 1);

    // 保存图片
    cv::imwrite(output_name, image);
    // saveMatAsBin(output_name, image);
}
