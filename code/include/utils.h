#ifndef UTILS_H
#define UTILS_H


#include <string>
#include <iostream>


class Timera
{
public:
    Timera() : beg_(clock_::now())
    {}
    
    void reset()
    {
        beg_ = clock_::now();
    }

    double elapsed() const
    {
        return std::chrono::duration_cast<second_>(clock_::now() - beg_).count();
    }

    void out(std::string message = "", int sensor = 0)
    {
        double t = elapsed();
        std::cout << message << " elapsed time: " << t << " ms" << std::endl;
        // LOG_I("{} cost {} ms", message, t);
        reset();
    }

    void out_(std::string message = "", int sensor = 0)
    {
        double t = elapsed();
        // LOG_I("{} cost {} ms", message, t);
        reset();
    }

    double get_duration() const
    {
        return elapsed();
    }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::milli> second_;
    std::chrono::time_point<clock_> beg_;
};

// class Utils 
// {
// private:
//     std::string name; // 实例属性

// public:
//     // 构造函数
//     Utils(const std::string& name);

//     // 打印实例相关信息
//     void printName() const;

//     // 静态方法
//     static void log(const std::string& message);

//     // 静态方法访问类信息
//     static void printClassInfo();
// };



#include <opencv2/opencv.hpp>
#include <string>
#include <cstdio>

inline bool saveMatAsBin(const std::string& filename, const cv::Mat& image) {
    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) return false;
    size_t bytesWritten = fwrite(image.data, 1, image.total() * image.elemSize(), fp);
    fclose(fp);
    return bytesWritten == image.total() * image.elemSize();
}

#endif // UTILS_H

