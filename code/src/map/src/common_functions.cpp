#include "common_functions.h"

#include <cmath>

// Function to rotate a point around a given center
cv::Point2f rotatePoint(const cv::Point2f& point, const cv::Point2f& center, float angle) {
    // Convert angle from degrees to radians
    // float rad = angle * CV_PI / 180.0;
    float rad = angle;

    // Create rotation matrix
    cv::Matx22f rotation_matrix(std::cos(rad), -std::sin(rad),
                                std::sin(rad),  std::cos(rad));

    // Translate point to origin (relative to center)
    cv::Point2f translated_point = point;
    // cv::Point2f translated_point = point - center;

    // Apply rotation
    cv::Point2f rotated_point = rotation_matrix * translated_point;

    // Translate back to original position
    return rotated_point + center;
}


cv::Point2f rotateAndTranslatePoint(const cv::Point2f& point, const cv::Mat& rotation_matrix, const cv::Point2f& offset) {
    return cv::Point2f(
        rotation_matrix.at<float>(0, 0) * point.x + rotation_matrix.at<float>(0, 1) * point.y,
        rotation_matrix.at<float>(1, 0) * point.x + rotation_matrix.at<float>(1, 1) * point.y
    ) + offset;
}

std::vector<int> get_unique_indexes(const std::vector<cv::Point>& points) 
{
    auto cmp = [](const cv::Point& a, const cv::Point& b) {
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    };
    std::set<cv::Point, decltype(cmp)> seen(cmp);
    std::vector<int> unique_indexes;

    for (size_t i = 0; i < points.size(); ++i) {
        if (seen.find(points[i]) == seen.end()) {
            seen.insert(points[i]);
            unique_indexes.push_back(i);
        }
    }
    return unique_indexes;
}


bool is_in_2m_range(const cv::Point& pt, const cv::Point& center, int  map_len) 
{
    return (pt.x >= center.x - map_len && pt.x <= center.x + map_len &&
            pt.y >= center.y - map_len && pt.y <= center.y + map_len);
}


cv::Point world2grids(const cv::Point3f& pt, float gird_size) 
{
    return cv::Point(
        static_cast<int>(std::round(pt.x / gird_size)),
        static_cast<int>(std::round(pt.y / gird_size))
        // static_cast<int>(std::round(pt.z / gird_size))
    );
}


// 返回两个vector<int>，分别是list1和list2中相同点的下标
std::pair<std::vector<int>, std::vector<int>> find_common_point_indices(const std::vector<cv::Point>& list1, const std::vector<cv::Point>& list2)
{
    std::vector<int> idx1, idx2;
    for (int i = 0; i < list1.size(); ++i) {
        for (int j = 0; j < list2.size(); ++j) {
            if (list1[i] == list2[j]) {
                idx1.push_back(i);
                idx2.push_back(j);
            }
        }
    }
    return {idx1, idx2};
}


std::vector<cv::Point> select_points_by_indices(const std::vector<cv::Point>& points,
                                                const std::vector<int>& indices)
{
    std::vector<cv::Point> result;
    for (int idx : indices) {
        if (idx >= 0 && idx < points.size()) {
            result.push_back(points[idx]);
        }
        // 如果要抛异常可以写 else throw std::out_of_range(...)
    }
    return result;
}


bool isValueInVector(const std::vector<int>& vec, int value) 
{
    for (int element : vec) {
        if (element == value) {
            return true;
        }
    }
    return false;
}


unsigned char updateProbability(unsigned char PO, float P_Z_given_O, float P_Z_given_not_O)
{
    float P_O = PO / 256.0f;

    float P_Z = P_Z_given_O * P_O + P_Z_given_not_O * (1.0f - P_O);
    float P_O_given_Z = (P_Z_given_O * P_O) / P_Z;

    int ans = int(P_O_given_Z * 256);

    // return (ans > 255) ? 255 : ans;
    if (ans > 229) return 229;
    if (ans < 23) return 23;
    return ans;

}



bool calculateAngle(const cv::Point& p1, const cv::Point& p2, float yaw, int vis_r, int vis_deg)
{
    // 计算坐标差
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    float theta_cam = yaw;
    
    // 计算径向距离 r
    float r = std::sqrt(dx * dx + dy * dy);

    if (r > vis_r) 
        return false;
    
    float theta_obs = std::atan2(dy, dx);
    float delta_theta = theta_obs - theta_cam;

    // 归一化到[-pi, pi]
    while(delta_theta > M_PI) delta_theta -= 2*M_PI;
    while(delta_theta < -M_PI) delta_theta += 2*M_PI;

    float fov_rad = vis_deg * M_PI / 180.0f;
    
    return std::abs(delta_theta) <= fov_rad / 2;

}



int pointToIndex(const cv::Point& pt, int width, int height) 
{
    return pt.x * height + pt.y;
}


bool is_in_2m_range(const cv::Point2f& center, const cv::Point2f& pt, int map_len) {
    // 定义上下左右的范围
    float left_bound = center.x - map_len;
    float right_bound = center.x + map_len;
    float top_bound = center.y + map_len;
    float bottom_bound = center.y - map_len;

    // 检查目标点是否在范围内
    return (pt.x >= left_bound && pt.x <= right_bound && 
            pt.y >= bottom_bound && pt.y <= top_bound);
}


std::vector<cv::Point> getPolygonInnerPoints(const std::vector<cv::Point>& frame_data_cur, const std::vector<int>& unique_indexes)
{
    // 初始化多边形顶点和边界值
    std::vector<cv::Point> vis_contour;
    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::min();
    int max_y = std::numeric_limits<int>::min();

    // 检查 unique_indexes 是否为空
    if (unique_indexes.empty()) {
        std::cerr << "Error: unique_indexes is empty." << std::endl;
        return {};
    }

    // 遍历索引，获取多边形顶点并计算边界
    for (size_t i = 0; i < unique_indexes.size(); i++) {
        if (unique_indexes[i] < 0 || unique_indexes[i] >= frame_data_cur.size()) {
            std::cerr << "Error: unique_indexes[" << i << "] is out of bounds." << std::endl;
            return {};
        }

        cv::Point tmp = frame_data_cur[unique_indexes[i]];

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

    // 将 frame_data_cur 中的点存储在 unordered_set 中以便快速查找
    std::unordered_set<cv::Point, PointHash, PointEqual> frame_data_set(frame_data_cur.begin(), frame_data_cur.end());

    // 遍历图像，获取多边形内部的栅格点
    std::vector<cv::Point> inner_pts;
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            uchar pixelValue = image.at<uchar>(y, x); // 获取像素值
            if (pixelValue > 0) { // 如果像素值大于 0，说明该点在多边形内部
                cv::Point point(x + min_x, y + min_y);
                if (frame_data_set.find(point) == frame_data_set.end()) { // 如果点不在 frame_data_cur 中
                    inner_pts.emplace_back(point);
                }
            }
        }
    }

    return inner_pts;
}


std::vector<cv::Point> getPolygonInnerPoints1(const std::set<cv::Point>& frame_data_cur)
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

    // 将 frame_data_cur 中的点存储在 unordered_set 中以便快速查找
    std::unordered_set<cv::Point, PointHash, PointEqual> frame_data_set(frame_data_cur.begin(), frame_data_cur.end());

    // 遍历图像，获取多边形内部的栅格点
    std::vector<cv::Point> inner_pts;
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            uchar pixelValue = image.at<uchar>(y, x); // 获取像素值
            if (pixelValue > 0) { // 如果像素值大于 0，说明该点在多边形内部
                cv::Point point(x + min_x, y + min_y);
                if (frame_data_set.find(point) == frame_data_set.end()) { // 如果点不在 frame_data_cur 中
                    inner_pts.emplace_back(point);
                }
            }
        }
    }

    return inner_pts;
}

