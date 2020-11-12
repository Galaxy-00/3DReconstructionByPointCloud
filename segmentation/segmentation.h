#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <opencv2/aruco/charuco.hpp>
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

using namespace std;

class Segmentation
{
public:
    // 使用的通道, 用于赋值cur_channel
    enum
    {
        H = 0,
        S = 1,
        V = 2,
        L = 3,
        A = 4,
        B = 5
    };

    // 构造函数
    Segmentation(int thresh_min, int thresh_max, int cur_channel, int cw);
    // 处理图像
    bool imageProcess(cv::Mat &inImg, cv::Mat &binaryImg, cv::Mat &resImg);

public:
    int thresh_min; // 最小阈值
    int thresh_max; // 最大阈值
    int cur_channel; // 当前所用图像通道
    int cw; // 通道阈值化方向(针对Hue)

private:
    // 获取所用通道的图像
    static cv::Mat getChannel(cv::Mat &inImg, int cur_channel); 
};

#endif // SEGMRNTATION_H