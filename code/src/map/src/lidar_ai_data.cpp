#include "lidar_ai_data.h"
#include <fstream>
#include <sstream>
#include <regex>


std::vector<LiDARFrame> loadLiDARData(const std::string& path) {
    std::ifstream file(path);
    std::vector<LiDARFrame> lidar_frames;
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open LiDAR file: " << path << std::endl;
        return lidar_frames;
    }

    LiDARFrame frame;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        if (line.find("time_stamp:") == 0) {
            if (frame.time_stamp != 0) {
                lidar_frames.push_back(frame);
                frame = LiDARFrame(); // 重置
            }
            frame.time_stamp = std::stoll(line.substr(11));
        }
        else if (line.find("fusepose:") == 0) {
            std::istringstream iss(line.substr(9));
            float x, y, z;
            iss >> x >> y >> z;
            frame.fusepose = cv::Point3f(x, y, z);
        }
        else if (line.find("rpy:") == 0) {
            std::istringstream iss(line.substr(4));
            float roll, pitch, yaw;
            iss >> roll >> pitch >> yaw;
            frame.rpy = cv::Point3f(roll, pitch, yaw);
        }
    }

    if (frame.time_stamp != 0) {
        lidar_frames.push_back(frame);
    }

    return lidar_frames;
}

std::vector<AIFrame> loadAIData(const std::string& path) {
    std::ifstream file(path);
    std::vector<AIFrame> ai_frames;
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open AI file: " << path << std::endl;
        return ai_frames;
    }

    AIFrame frame;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        if (line.find("time_stamp:") == 0) {
            if (frame.time_stamp != 0) {
                ai_frames.push_back(frame);
                frame = AIFrame();
            }
            frame.time_stamp = std::stoll(line.substr(11));
        }
        else if (line.find("xyz:") == 0) {
            std::istringstream iss(line.substr(4));
            float x, y, z;
            iss >> x >> y >> z;
            frame.xyz.emplace_back(x, y, z);
        }
    }

    if (frame.time_stamp != 0) {
        ai_frames.push_back(frame);
    }

    return ai_frames;
}

std::vector<MatchedFrame> matchingPoseWithAI(const std::vector<AIFrame>& ai_data, const std::vector<LiDARFrame>& li_data) {
    std::vector<MatchedFrame> result;
    size_t i = 0, j = 0;
    const long long threshold = 50; // 时间戳匹配阈值
    size_t index = 0;

    while (i < li_data.size() && j < ai_data.size()) {
        long long diff = li_data[i].time_stamp - ai_data[j].time_stamp;

        if (std::abs(diff) < threshold) {
            if (!ai_data[j].xyz.empty()) {
                // std::cout << index << " " << ai_data[j].time_stamp << std::endl;
            }

            MatchedFrame tmp;
            tmp.lidar_time_stamp = li_data[i].time_stamp;
            tmp.ai_time_stamp = ai_data[j].time_stamp;
            tmp.fusepose = li_data[i].fusepose;
            tmp.rpy = li_data[i].rpy;
            tmp.xyz = ai_data[j].xyz;
            tmp.diff = diff;
            tmp.pcl_no = ai_data[j].xyz.size();

            result.push_back(tmp);
            index++;
            j++; // 移动 AI 数据指针
        } else if (diff < 0) {
            i++; // LiDAR 数据时间戳较小，移动 LiDAR 数据指针
        } else {
            j++; // AI 数据时间戳较小，移动 AI 数据指针
        }
    }

    return result;
}


void findMinMaxXY(const std::vector<MatchedFrame>& frames, 
                  float& min_x, float& max_x, 
                  float& min_y, float& max_y) {
    min_x = min_y = std::numeric_limits<float>::max();
    max_x = max_y = std::numeric_limits<float>::lowest();

    for (const auto& frame : frames) {
        auto pt = frame.fusepose;
        {
            if (pt.x < min_x) min_x = pt.x;
            if (pt.x > max_x) max_x = pt.x;
            if (pt.y < min_y) min_y = pt.y;
            if (pt.y > max_y) max_y = pt.y;
        }
    }
}


std::vector<MatchedFrame> readAndParseData(const std::string& file_name) {
    std::ifstream file(file_name);
    std::string line;
    std::vector<MatchedFrame> formatted_frames;

    if (!file.is_open()) {
        std::cerr << "無法打開文件: " << file_name << std::endl;
        return formatted_frames; // 返回空的向量
    }

    long long lidar_time_stamp = 0;
    long long ai_time_stamp = 0;
    cv::Point3f fusepose;
    cv::Point3f rpy;

    while (std::getline(file, line)) {
        std::smatch match;

        if (line.find("fusepose:") != std::string::npos) {
            std::regex fusepose_regex(R"(([-+]?\d*\.\d+|[-+]?\d+))");
            std::vector<float> fusepose_data;
            auto words_begin = std::sregex_iterator(line.begin(), line.end(), fusepose_regex);
            auto words_end = std::sregex_iterator();

            for (std::sregex_iterator i = words_begin; i != words_end; ++i) {
                fusepose_data.push_back(std::stof((*i).str()));
            }

            lidar_time_stamp = static_cast<long long>(fusepose_data[0]);
            fusepose = cv::Point3f(fusepose_data[1], fusepose_data[2], fusepose_data[3]);
            rpy = cv::Point3f(fusepose_data[4], fusepose_data[5], fusepose_data[6]);
        } 
        else if (line.find("ai_time:") != std::string::npos) {
            std::regex ai_time_regex(R"(\d+)");
            std::smatch ai_time_match;
            if (std::regex_search(line, ai_time_match, ai_time_regex)) {
                ai_time_stamp = std::stoll(ai_time_match.str());
            }
        } 
        else if (line.find("ai_obts:") != std::string::npos) {
            std::string ai_obts_data = line.substr(line.find(":") + 1);
            std::vector<cv::Point3f> xyz;
            std::regex ai_obts_regex(R"(\((.*?)\))");
            auto ai_obts_begin = std::sregex_iterator(ai_obts_data.begin(), ai_obts_data.end(), ai_obts_regex);
            auto ai_obts_end = std::sregex_iterator();

            for (std::sregex_iterator i = ai_obts_begin; i != ai_obts_end; ++i) {
                std::string point_str = (*i)[1].str();
                std::istringstream point_stream(point_str);
                std::string coord;
                std::vector<float> coords;

                while (std::getline(point_stream, coord, ',')) {
                    coords.push_back(std::stof(coord));
                }

                if (coords.size() == 3) {
                    if((coords[0] < 1.5f && coords[0] > 0.0f )&& (coords[1] < 1.5f && coords[1] > -1.5f) )
                        xyz.emplace_back(coords[0], coords[1], coords[2]);
                }
            }

            size_t pcl_no = xyz.size();
            long long diff = ai_time_stamp - lidar_time_stamp;

            // 將格式化數據添加到列表
            MatchedFrame frame = {
                lidar_time_stamp,
                ai_time_stamp,
                fusepose,
                rpy,
                xyz,
                diff,
                pcl_no
            };

            formatted_frames.push_back(frame);
        }
    }

    file.close();
    return formatted_frames;
}


std::vector<std::vector<cv::Point>> findValidContours1(const cv::Mat& mask_img, int mini_area, bool bSave)
{
    if (mask_img.empty()) {
        std::cerr << "Error: The input mask image is empty!" << std::endl;
        return {};
    }
    std::vector<std::vector<cv::Point>> validContours;

    // Ensure the input is a 3-channel RGB image
    if (mask_img.type() != CV_8UC3) {
        std::cerr << "Error: The input image is not a 3-channel RGB image!" << std::endl;
        return {};
    }
    // Convert the RGB image to grayscale
    cv::Mat gray;
    cv::cvtColor(mask_img, gray, cv::COLOR_BGR2GRAY);
    // Apply binary thresholding to create a binary mask
    cv::Mat binary;
    // cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    #if bSave
        cv::Mat all_contours = mask_img.clone();
        // Generate random colors for each contour
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dist(0, 255);
    
        for (size_t i = 0; i < contours.size(); ++i) {
            // Generate a random color
            cv::Scalar random_color(dist(gen), dist(gen), dist(gen));
    
            // Draw the current contour on the mask_img
            cv::drawContours(all_contours, contours, static_cast<int>(i), random_color, 2, cv::LINE_8);
        }
        cv::imwrite("all_contours.png", all_contours);

    #endif

    // If no contours are found, return an empty list
    if (contours.empty()) {
        std::cerr << "Error: No contours found in the image!" << std::endl;
        return {};
    }

    // Find the largest contour based on area
    // auto largestContour = *std::max_element(
    //     contours.begin(), contours.end(),
    //     [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
    //         return cv::contourArea(a) < cv::contourArea(b);
    //     });
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > mini_area) {
            validContours.push_back(contour);
        }
    }

    #if bSave
        cv::Mat only_img = mask_img.clone();
        cv::Scalar color(0, 255, 0); // Green color
        for(auto& contour : contours)
        {
            cv::drawContours(only_img, std::vector<std::vector<cv::Point>>{contour}, -1, color, 2, cv::LINE_8);
        }
        cv::imwrite("only_contours.png", only_img);

    #endif 

    // Return the largest contour as a vector of (x, y) points
    return validContours;
}


std::vector<std::vector<cv::Point>> findValidContours2(const cv::Mat& mask_img, 
    int map_index, 
    bool bSave,
    int mini_area
    )
{
    if (mask_img.empty()) {
        std::cerr << "Error: The input mask image is empty!" << std::endl;
        return {};
    }
    std::vector<std::vector<cv::Point>> validContours;

    // Ensure the input is a 3-channel RGB image
    if (mask_img.type() != CV_8UC3) {
        std::cerr << "Error: The input image is not a 3-channel RGB image!" << std::endl;
        return {};
    }
    // Convert the RGB image to grayscale
    cv::Mat gray;
    cv::cvtColor(mask_img, gray, cv::COLOR_BGR2GRAY);
    // Apply binary thresholding to create a binary mask
    cv::Mat binary;
    // cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);
    // cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV);
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY);

    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    if(bSave){
        // cv::Mat all_contours = mask_img.clone();
        cv::Mat all_contours = mask_img.clone();
        // Generate random colors for each contour
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dist(0, 255);
    
        for (size_t i = 0; i < contours.size(); ++i) {
            // Generate a random color
            // cv::Scalar random_color(dist(gen), dist(gen), dist(gen));
            cv::Scalar random_color(255, 0, 0);
    
            // Draw the current contour on the mask_img
            cv::drawContours(all_contours, contours, static_cast<int>(i), random_color, 2, cv::LINE_8);
        }
        std::string save_all = "/media/ai/5E8C-B109/record_ai/0041/x86/perception/all_" + std::to_string(map_index) + ".png";

        cv::imwrite(save_all, all_contours);

    }

    // If no contours are found, return an empty list
    if (contours.empty()) {
        std::cerr << "Error: No contours found in the image!" << std::endl;
        return {};
    }

    // Find the largest contour based on area
    // auto largestContour = *std::max_element(
    //     contours.begin(), contours.end(),
    //     [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
    //         return cv::contourArea(a) < cv::contourArea(b);
    //     });
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > mini_area) {
            validContours.push_back(contour);
        }
    }

    if(bSave){
        cv::Mat only_img = mask_img.clone();
        cv::Scalar color(255, 0, 0); // Green color
        for(auto& contour : contours)
        {
            cv::drawContours(only_img, std::vector<std::vector<cv::Point>>{contour}, -1, color, 5, cv::LINE_8);
        }
        std::string only_path = "/media/ai/5E8C-B109/record_ai/0041/x86/perception/only_" + std::to_string(map_index) + ".png";

        cv::imwrite(only_path, only_img);

    }
    // Return the largest contour as a vector of (x, y) points
    return validContours;    
}



std::vector<cv::Point> findValidContours(const cv::Mat& mask_img)
{
    if (mask_img.empty()) {
        std::cerr << "Error: The input mask image is empty!" << std::endl;
        return {};
    }

    // Ensure the input is a 3-channel RGB image
    if (mask_img.type() != CV_8UC3) {
        std::cerr << "Error: The input image is not a 3-channel RGB image!" << std::endl;
        return {};
    }
    // Convert the RGB image to grayscale
    cv::Mat gray;
    cv::cvtColor(mask_img, gray, cv::COLOR_BGR2GRAY);
    // Apply binary thresholding to create a binary mask
    cv::Mat binary;
    // cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    #if 1
        cv::Mat all_contours = mask_img.clone();
        // Generate random colors for each contour
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dist(0, 255);
    
        for (size_t i = 0; i < contours.size(); ++i) {
            // Generate a random color
            cv::Scalar random_color(dist(gen), dist(gen), dist(gen));
    
            // Draw the current contour on the mask_img
            cv::drawContours(all_contours, contours, static_cast<int>(i), random_color, 2, cv::LINE_8);
        }
        cv::imwrite("all_contours.png", all_contours);

    #endif

    // If no contours are found, return an empty list
    if (contours.empty()) {
        std::cerr << "Error: No contours found in the image!" << std::endl;
        return {};
    }

    // Find the largest contour based on area
    auto largestContour = *std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });

    #if 1
        cv::Mat only_img = mask_img.clone();
        cv::Scalar color(0, 255, 0); // Green color
        cv::drawContours(only_img, std::vector<std::vector<cv::Point>>{largestContour}, -1, color, 2, cv::LINE_8);
        cv::imwrite("only_contours.png", only_img);

    #endif 

    // Return the largest contour as a vector of (x, y) points
    return largestContour;
}

std::vector<std::vector<cv::Point>> interpolate_polygon1(const std::vector<std::vector<cv::Point>>& polygon, int max_distance)
{
    std::vector<std::vector<cv::Point>> res;
    for(auto& contours: polygon )
    {
        res.push_back(interpolate_polygon(contours));
    }
    return res;
}



std::vector<cv::Point> interpolate_polygon(const std::vector<cv::Point>& polygon, int max_distance) 
{
    std::vector<cv::Point> interpolated_polygon;

    for (size_t i = 0; i < polygon.size(); ++i) {
        cv::Point p1 = polygon[i];
        cv::Point p2 = polygon[(i + 1) % polygon.size()]; // Loop back to the first point

        // Add the first point to the result
        interpolated_polygon.push_back(p1);

        // Calculate the Euclidean distance between the two points
        double distance = cv::norm(p2 - p1);

        // 计算 y 值的权重
        // double y_weight = (static_cast<double>(p1.y + p2.y) / 2.0) / 320.0; // 将 y 值归一化到 [0, 1]

        // // 根据距离和 y 值计算插值数量
        // int num_points = static_cast<int>((distance / 10.0) + (y_weight * 10)); // 插值数量公式
        int num_points = static_cast<int>(distance / 5.0f) + 1;
        // 生成插值点
        for (int i = 1; i <= num_points; ++i) {
            double t = static_cast<double>(i) / (num_points + 1); // 插值比例
            int x = static_cast<int>(p1.x + t * (p2.x - p1.x));
            int y = static_cast<int>(p1.y + t * (p2.y - p1.y));
            interpolated_polygon.push_back(cv::Point(x, y));
        }





    }

    return interpolated_polygon;
}

/**
 * 判断点是否在水平直线 y = b 的下方
 * @param y 点的 y 坐标
 * @param b 水平直线的 y 值
 * @return 如果点在直线下方，返回 true；否则返回 false
 */
bool isBelowLine(int y, int b) {
    return y > b; // 在像素坐标系中，y 越大表示越靠下
}

/**
 * 计算线段 p1-p2 与水平直线 y = b 的交点
 * @param p1 线段的起点
 * @param p2 线段的终点
 * @param b 水平直线的 y 值
 * @return 如果有交点，返回交点；如果没有交点，返回 cv::Point(-1, -1)
 */
cv::Point lineIntersection(const cv::Point& p1, const cv::Point& p2, int b) {
    // 如果线段平行于水平直线
    if (p1.y == p2.y) {
        return cv::Point(-1, -1); // 返回特殊值表示没有交点
    }

    // 计算交点
    double t = (b - p1.y) / static_cast<double>(p2.y - p1.y);
    if (t >= 0.0 && t <= 1.0) {
        int x = static_cast<int>(p1.x + t * (p2.x - p1.x));
        return cv::Point(x, b); // 返回交点
    }

    return cv::Point(-1, -1); // 返回特殊值表示没有交点
}


/**
 * 对点列表进行插值，确保相邻点之间的距离不超过 maxDistance
 * @param points 原始点列表
 * @param maxDistance 两点之间的最大距离
 * @return 插值后的点列表
 */
std::vector<cv::Point> interpolatePoints(const std::vector<cv::Point>& points, int maxDistance) {
    std::vector<cv::Point> interpolatedPoints;

    if (points.empty()) {
        return interpolatedPoints; // 如果点列表为空，直接返回空列表
    }

    for (size_t i = 0; i < points.size() - 1; ++i) {
        cv::Point p1 = points[i];
        cv::Point p2 = points[i + 1];
        interpolatedPoints.push_back(p1);

        // 计算两点之间的欧几里得距离
        double distance = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));

        // 如果距离超过阈值，则进行插值
        if (distance > maxDistance) {
            int numPoints = static_cast<int>(distance / maxDistance);
            for (int j = 1; j <= numPoints; ++j) {
                double t = static_cast<double>(j) / (numPoints + 1);
                int x = static_cast<int>(p1.x + t * (p2.x - p1.x));
                int y = static_cast<int>(p1.y + t * (p2.y - p1.y));
                interpolatedPoints.push_back(cv::Point(x, y));
            }
        }
    }
    interpolatedPoints.push_back(points.back()); // 添加最后一个点
    return interpolatedPoints;
}

/**
 * 获取轮廓和水平直线 y = b 相交的下半部分，并将直线上的点加入形成闭合曲线
 * @param polygon 轮廓点列表，类型为 std::vector<cv::Point>
 * @param lineB 水平直线的 y 值
 * @param step 水平直线交点的采样间隔
 * @param maxDistance 水平直线交点插值的最大距离
 * @return 闭合曲线的点列表，类型为 std::vector<cv::Point>
 */
// std::vector<cv::Point> getLowerPart(const std::vector<cv::Point>& polygon, int lineB = 348, int step = 2, int maxDistance = 5.0); // 338
std::vector<cv::Point> getLowerPart(const std::vector<cv::Point>& polygon, int lineB, int step, int maxDistance) {
    std::vector<cv::Point> lowerPart;

    // 遍历多边形的每条边，寻找交点和位于水平线下方的点
    for (size_t i = 0; i < polygon.size(); ++i) {
        cv::Point p1 = polygon[i];
        cv::Point p2 = polygon[(i + 1) % polygon.size()]; // 处理多边形的最后一条边

        // 如果当前点在水平线下方，直接加入结果
        if (isBelowLine(p1.y, lineB)) {
            lowerPart.push_back(p1);
        }

        // 计算线段与水平线的交点
        cv::Point intersection = lineIntersection(p1, p2, lineB);
        if (intersection != cv::Point(-1, -1)) { // 如果交点有效，加入结果
            std::cout << "p1: " << p1.x << " " << p1.y << std::endl;
            std::cout << "p2: " << p2.x << " " << p2.y << std::endl;
            std::cout << "wlm: \n";
            lowerPart.push_back(intersection);
        }
    }

    // 提取水平线上的交点并按 x 坐标排序
    std::vector<cv::Point> linePoints;
    for (const auto& point : lowerPart) {
        if (point.y == lineB) {
            linePoints.push_back(point);
        }
    }
    sort(linePoints.begin(), linePoints.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.x < b.x;
    });

    // 按步长采样水平线上的点
    std::vector<cv::Point> sampledLinePoints;
    for (size_t i = 0; i < linePoints.size(); i += step) {
        sampledLinePoints.push_back(linePoints[i]);
    }

    // 对采样点进行插值，确保相邻点之间的距离不超过 maxDistance
    sampledLinePoints = interpolatePoints(sampledLinePoints, maxDistance);

    // 将多边形下半部分的点和水平线上的点合并
    lowerPart.insert(lowerPart.end(), sampledLinePoints.begin(), sampledLinePoints.end());

    return lowerPart;
}



cv::Point3f uv_to_world(float u, float v, const cv::Mat& K, const cv::Mat& T_w_cam) {
    // Step 1: 计算 K 的逆矩阵
    cv::Mat K_inv = K.inv();
    
    // 从 T_w_cam 中提取旋转矩阵 R 和平移向量 t
    cv::Mat R = T_w_cam(cv::Range(0, 3), cv::Range(0, 3)); // 3x3 旋转矩阵
    cv::Mat t = T_w_cam(cv::Range(0, 3), cv::Range(3, 4)); // 3x1 平移向量

    // Step 2: 归一化坐标
    cv::Mat uv1 = (cv::Mat_<float>(3, 1) << u, v, 1.0); // 构造点 (u, v, 1)
    cv::Mat xnyn1 = K_inv * uv1; // 计算归一化坐标
    float xn = xnyn1.at<float>(0, 0); // xn
    float yn = xnyn1.at<float>(1, 0); // yn

    // Step 3: 计算尺度因子 s
    float denom = R.at<float>(2, 0) * xn + R.at<float>(2, 1) * yn + R.at<float>(2, 2);
    float s = -t.at<float>(2, 0) / denom;

    // Step 4: 计算相机坐标系下的点
    cv::Mat xyz_cam = (cv::Mat_<float>(3, 1) << xn * s, yn * s, s);

    // Step 5: 转换到世界坐标系
    cv::Mat xyz_world = R * xyz_cam + t;

    // 返回世界坐标系下的点，使用 cv::Point3f 表示
    return cv::Point3f(xyz_world.at<float>(0, 0), xyz_world.at<float>(1, 0), xyz_world.at<float>(2, 0));
}