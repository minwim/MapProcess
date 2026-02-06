#include "data_frame.h"
#include "common_functions.h"

// 构造函数
FramesData::FramesData(cv::Point3f fusepose_xyz, cv::Point3f fusepose_rpy, 
                       const std::vector<std::vector<cv::Point3f>>& ai_obts,
                       const std::vector<std::vector<cv::Point3f>>& inner_obts,
                       float max_x, float max_y,
                       int ai_len, int inner_len, int map_index)
    : fusepose_xyz_(fusepose_xyz), fusepose_rpy_(fusepose_rpy),
      ai_obts_(ai_obts), inner_obts_(inner_obts),
      max_x_(max_x), max_y_(max_y),
      ai_len_(ai_len), inner_len_(inner_len), map_index_(map_index),
      frame_width_(100), frame_height_(100),
      GRID_SIZE(0.05f), LOCAL_CENTER(50) {
    
        //unkown
        perceptionMap_ = cv::Mat(frame_height_, frame_width_, CV_8UC1, cv::Scalar(128));
        world_grid_obts_.resize(ai_len_ );
        local_grid_obts_.resize(ai_len_ );

}

// 初始化函数
void FramesData::Init() {

    calcCamLocalPose();
    
    // handleCureFramesData();
}

// 计算小车在局部地图中的位置
void FramesData::calcCamLocalPose() {
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
void FramesData::handleCureFramesData() 
{
    // step1: Get all grass contours:
    for (size_t i = 0; i < ai_obts_.size(); ++i) 
    {
        auto pts = ai_obts_[i];
        transformPoints(pts, i);
    }

    // step2: drawContours
    auto contours = setToVector(local_grid_obts_);

    // draw exter_contours point
    #if 0
        cv::Mat externImg = cv::Mat(frame_height_, frame_width_, CV_8UC1, cv::Scalar(0));

        for (size_t i = 0; i < contours.size(); i++)
        {
            if(i == 1) continue;
            for (size_t j = 0; j < contours[i].size(); j++)
            {
                auto point = contours[i][j];
                // std::cout << "p: " << point.x << " " << point.y << std::endl;
                // cv::circle(externImg, point, 1, cv::Scalar(255), cv::FILLED); // 半径为 1，填充颜色
                externImg.at<unsigned char>(point.y , point.x) = 128 * i + 90;
            }
            
        }
        std::string fileName = "exter_" + std::to_string(map_index_) + ".png";

        cv::imwrite(fileName, externImg);
    #endif
    
    int contour_num  = contours.size();
    // LI-DAI-TAO-JIANG
    if(contour_num > 1)
    {
        // last
        cv::drawContours(perceptionMap_, contours, static_cast<int>(contours.size() - 1), cv::Scalar(255), cv::FILLED);
        for (size_t i = 0; i < contours.size()-1; i++) {
            // cv::Mat mask11 = cv::Mat::zeros(perceptionMap_.size(), CV_8UC1);
            cv::drawContours(perceptionMap_, contours, int(i), cv::Scalar(0), cv::FILLED);
        }
    }
    else{
        // TODO wlm
        // cv::drawContours(perceptionMap_, contours, static_cast<int>(0), cv::Scalar(0), cv::FILLED);
        cv::drawContours(perceptionMap_, contours, static_cast<int>(0), cv::Scalar(255), cv::FILLED);

    }

    #if 0
        // draw visual
        std::string fileName = "project_" + std::to_string(map_index_) + ".png";
        // cv::imwrite(fileName, perceptionMap_);

        // draw car
        cv::Mat  carImg = perceptionMap_.clone();

        auto p1 = cv::Point(car_pose_[1].y, car_pose_[1].x); // 调换 x 和 y
        auto p2 = cv::Point(car_pose_[2].y, car_pose_[2].x);
        auto p3 = cv::Point(car_pose_[3].y, car_pose_[3].x);
        auto p4 = cv::Point(car_pose_[4].y, car_pose_[4].x);
        auto c1 = cv::Point(car_pose_[5].y, car_pose_[5].x);
        cv::line(carImg, p1, p2, cv::Scalar(255), 1);
        cv::line(carImg, p2, p3, cv::Scalar(255), 1);
        cv::line(carImg, p3, p4, cv::Scalar(255), 1);
        cv::line(carImg, p4, p1, cv::Scalar(255), 1);

        // cv::circle(image, m1, 1, cv::Scalar(255, 0, 0), 1);
        cv::circle(carImg, c1, 1, cv::Scalar(255), 1);

        cv::imwrite(fileName, carImg);


    #endif
    
    



}

    //
 



// 转换点集到世界坐标和局部栅格坐标
void FramesData::transformPoints(const std::vector<cv::Point3f>& pts, int contour_index)
{
    for (const auto& p : pts) {
        transformCoords(p, contour_index);
    }
}

// 坐标转换函数
void FramesData::transformCoords(const cv::Point3f& local_points, int contour_index) {
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
    world_grid_obts_[contour_index].insert(cv::Point(global_obts_grid_x, global_obts_grid_y));
    local_grid_obts_[contour_index].insert(cv::Point(local_grid_x, local_grid_y));

    // std::cout << "xylocal: " << x_local << " " << y_local << "local_grid_xy: " << local_grid_x << " " << local_grid_y << std::endl;
}

std::vector<std::vector<cv::Point>> FramesData::setToVector(const std::vector<OrderedUniquePoints>& set_data)
{
    
    std::vector<std::vector<cv::Point>> contours;
    contours.reserve(set_data.size());
    for (const auto& oup : set_data)
    {
        contours.push_back(oup.data());
    }
    return contours;
}


std::vector<cv::Point> FramesData::getPolygonInnerPoints1(const std::set<cv::Point, PointComparator>& frame_data_cur)
{
    // 初始化多边形顶点和边界值
    std::vector<cv::Point> vis_contour;
    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::min();
    int max_y = std::numeric_limits<int>::min();

    size_t i = 0;
    for (auto it = frame_data_cur.begin(); it != frame_data_cur.end(); ++it, ++i)
    {
        cv::Point tmp = *it;
        // 更新边界值
        min_x = std::min(min_x, tmp.x);
        min_y = std::min(min_y, tmp.y);
        max_x = std::max(max_x, tmp.x);
        max_y = std::max(max_y, tmp.y);

        vis_contour.emplace_back(tmp);
    }

    // 检查 vis_contour 是否为空
    if (vis_contour.empty()) {
        std::cerr << "Error: vis_contour is empty." << std::endl;
        return {};
    }

    // 检查边界值是否有效
    if (max_x < min_x || max_y < min_y) {
        std::cerr << "Error: Invalid bounding box dimensions." << std::endl;
        return {};
    }

    // 将多边形顶点平移到以 (min_x, min_y) 为原点的坐标系
    for (size_t i = 0; i < vis_contour.size(); i++) {
        vis_contour[i].x -= min_x;
        vis_contour[i].y -= min_y;
    }

    // 创建一个空白图像，用于填充多边形
    cv::Mat image = cv::Mat::zeros(max_y - min_y + 1, max_x - min_x + 1, CV_8UC1);

    // 使用 fillPoly 填充多边形
    std::vector<std::vector<cv::Point>> polygons = {vis_contour};
    try {
        fillPoly(image, polygons, cv::Scalar(255)); // 填充值为 255
    } catch (const cv::Exception& e) {
        std::cerr << "Error in fillPoly: " << e.what() << std::endl;
        return {};
    }

    // 遍历图像，获取多边形内部的栅格点
    std::vector<cv::Point> inner_pts;
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            uchar pixelValue = image.at<uchar>(y, x); // 获取像素值
            if (pixelValue > 0) { // 如果像素值大于 0，说明该点在多边形内部
                cv::Point point(x + min_x, y + min_y);
                if (frame_data_cur.find(point) == frame_data_cur.end()) { // 如果点不在 frame_data_cur 中
                    inner_pts.emplace_back(point);
                }
            }
        }
    }

    return inner_pts;
}
