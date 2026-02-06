#include "data_lidar.h"
#include "common_functions.h"

// 构造函数
FramesLidar::FramesLidar(cv::Point3f fusepose_xyz, cv::Point3f fusepose_rpy, 
                        const std::vector<cv::Point3f> pts,
                        float max_x, float max_y,
                        int map_index)
    : fusepose_xyz_(fusepose_xyz), fusepose_rpy_(fusepose_rpy), pts_(pts),
      max_x_(max_x), max_y_(max_y),
      map_index_(map_index),
      frame_width_(100), frame_height_(100),
      GRID_SIZE(0.05f), LOCAL_CENTER(50) {
    
        //unkown
    perceptionMap_ = cv::Mat(frame_height_, frame_width_, CV_8UC1, cv::Scalar(128));
    // world_grid_obts_.resize(1);
    // local_grid_obts_.resize(1);

}

// 初始化函数
void FramesLidar::Init() {

    calcCamLocalPose();
    
    // handleCureFramesLidar();
}

// 计算小车在局部地图中的位置
void FramesLidar::calcCamLocalPose() {
    float yaw = fusepose_rpy_.z;
    int car_len = 666;
    int car_wid = 472;
    int car_cen = 513;
    float grid_size = 1 / GRID_SIZE;

    // x_top x_bottom y_left y_right 相对鱼世界坐标系
    int x_top = static_cast<int>((car_len - car_cen) / 1000.0f * grid_size);
    int x_bottom = static_cast<int>(-car_cen / 1000.0f * grid_size);
    int y_left = static_cast<int>(car_wid / 1000.0f / 2 * grid_size);
    int y_right = static_cast<int>(-car_wid / 1000.0f / 2 * grid_size);

    cv::Point2f p1(x_top, y_left);
    cv::Point2f p2(x_top, y_right);
    cv::Point2f p3(x_bottom, y_right);
    cv::Point2f p4(x_bottom, y_left);

    cv::Mat rotation_matrix = (cv::Mat_<float>(2, 2) << std::cos(yaw), -std::sin(yaw),
                                                         std::sin(yaw),  std::cos(yaw));
    int delta_size = frame_height_ / 2;
    cv::Point offset(delta_size, delta_size);
    // cv::Point offset(0, 0);

    // car pose on world-coordinate
    cv::Point rotated_p1 = rotateAndTranslatePoint(p1, rotation_matrix, offset);
    cv::Point rotated_p2 = rotateAndTranslatePoint(p2, rotation_matrix, offset);
    cv::Point rotated_p3 = rotateAndTranslatePoint(p3, rotation_matrix, offset);
    cv::Point rotated_p4 = rotateAndTranslatePoint(p4, rotation_matrix, offset);

    //un-used
    int cex = int(frame_width_ / 2);
    cv::Point centerxy = cv::Point(cex, cex);

    // convert
    cv::Point rp1 = cv::Point2f(rotated_p1.y, rotated_p1.x);
    cv::Point rp2 = cv::Point2f(rotated_p2.y, rotated_p2.x);
    cv::Point rp3 = cv::Point2f(rotated_p3.y, rotated_p3.x);
    cv::Point rp4 = cv::Point2f(rotated_p4.y, rotated_p4.x);

    car_pose_.push_back(centerxy);
    car_pose_.push_back(rp1);
    car_pose_.push_back(rp2);
    car_pose_.push_back(rp3);
    car_pose_.push_back(rp4);
    car_pose_.push_back(centerxy); // 旋转中心

    //
    int global_obts_grid_u = static_cast<int>((max_y_ - fusepose_xyz_.y) / GRID_SIZE);
    int global_obts_grid_v = static_cast<int>((max_x_ - fusepose_xyz_.x) / GRID_SIZE);
    
    gxy_ = cv::Point(global_obts_grid_u, global_obts_grid_v);

    cv::Point global_offest = gxy_ - offset;

    car_gpose_.push_back(gxy_);
    car_gpose_.push_back(rp1 + global_offest);
    car_gpose_.push_back(rp2 + global_offest);
    car_gpose_.push_back(rp3 + global_offest);
    car_gpose_.push_back(rp4 + global_offest);
    car_gpose_.push_back(gxy_);

    // std::cout << " car: " <<  rotated_p1.x << " " << rotated_p1.y << std::endl;
    // std::cout << " bal: " <<  gxy_.x << " " << gxy_.y << std::endl;
    // std::cout << "gcar: " <<  car_gpose_[1].x << " " << car_gpose_[1].y << std::endl;


}

// 处理当前帧数据
void FramesLidar::handleCureFramesLidar() 
{
    // step1: Get all grass contours:
    for (size_t i = 0; i < pts_.size(); ++i) 
    {
        auto pts = pts_[i];
        transformCoords(pts, i);
    }

    for (const auto& pt : local_grid_obts_) {
        // pt 是一个 const cv::Point&
        // std::cout << "x=" << pt.x << ", y=" << pt.y << std::endl;
        perceptionMap_.at<int>(pt.y, pt.x) = 1;

    }





}

 


// 转换点集到世界坐标和局部栅格坐标
void FramesLidar::transformPoints(const std::vector<cv::Point3f>& pts, int contour_index)
{
    for (const auto& p : pts) {
        transformCoords(p, contour_index);
    }
}

// 坐标转换函数
void FramesLidar::transformCoords(const cv::Point3f& local_points, int contour_index) {
    // cv::Point2f rotated_point = convertToWorldCoords(local_points);

    float x_local = local_points.x;
    float y_local = local_points.y;
    float x = fusepose_xyz_.x;
    float y = fusepose_xyz_.y;
    float yaw = fusepose_rpy_.z;


    cv::Point2f local_point(x_local, y_local);

    cv::Point2f center_point(x, y);
    cv::Point2f rotated_point = rotatePoint(local_point, center_point, yaw);

    float x_world = rotated_point.x;
    float y_world = rotated_point.y;


    //cvt
    int global_obts_grid_y = static_cast<int>((max_x_ - x_world) / GRID_SIZE);
    int global_obts_grid_x = static_cast<int>((max_y_ - y_world) / GRID_SIZE);

    //cvt
    int centery = static_cast<int>((max_x_ - x) / GRID_SIZE);
    int centerx = static_cast<int>((max_y_ - y) / GRID_SIZE);

    int local_grid_x = global_obts_grid_x - centerx + LOCAL_CENTER;
    int local_grid_y = global_obts_grid_y - centery + LOCAL_CENTER;

    // std::cout << "local:xy:  " << local_grid_x << " " << local_grid_y << std::endl;
    world_grid_obts_.insert(cv::Point(global_obts_grid_x, global_obts_grid_y));
    local_grid_obts_.insert(cv::Point(local_grid_x, local_grid_y));

    // std::cout << "xylocal: " << x_local << " " << y_local << "local_grid_xy: " << local_grid_x << " " << local_grid_y << std::endl;
}

