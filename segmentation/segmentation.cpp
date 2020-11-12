#include "segmentation.h"

Segmentation::Segmentation(int thresh_min, int thresh_max, int cur_channel, int cw) : thresh_max(thresh_max), thresh_min(thresh_min), cur_channel(cur_channel), cw(cw)
{
}

bool Segmentation::imageProcess(cv::Mat &inImg, cv::Mat &binaryImg, cv::Mat &resImg)
{
    cv::Mat channel = getChannel(inImg, cur_channel);
    if (cw)
    {
        binaryImg = channel > thresh_min & channel < thresh_max;
    }
    else
    {
        binaryImg = channel<thresh_min | channel> thresh_max;
    }

    // 分割出色卡里最大物体轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> heirachy;
    cv::Mat image_for_contours = binaryImg.clone();
    // 通过二值图像计算轮廓
    cv::findContours(image_for_contours, contours, heirachy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    double max_area = -1;
    int max_cont_idx = -1;
    int object_cont_idx = -1;

    if (contours.size() != 0)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            double t_area = cv::contourArea(contours[i]);
            if (t_area > max_area) // 找出最大的轮廓
            {
                max_area = t_area;
                max_cont_idx = i;
            }
        }

        // 最大颜色轮廓有儿子
        if (heirachy[max_cont_idx][2] != -1)
        {
            // 直接floodfill莽, 泛洪法求出resImg
            resImg = cv::Mat(inImg.rows + 2, inImg.cols + 2, CV_8UC1, cv::Scalar(0));
            binaryImg.copyTo(resImg(cv::Rect(1, 1, inImg.cols, inImg.rows)));
            cv::Rect cc;
            cv::floodFill(resImg, cv::Point(0, 0), cv::Scalar(255), &cc, cv::Scalar(128), cv::Scalar(128), 8);
            resImg = resImg(cv::Rect(1, 1, inImg.cols, inImg.rows)).clone();
            return true;
        }
    }

    resImg = cv::Mat(inImg.rows, inImg.cols, CV_8UC1, cv::Scalar(255));
    return false;
}

cv::Mat Segmentation::getChannel(cv::Mat &inImg, int cur_channel)
{
    assert(0 <= cur_channel && cur_channel <= 5);

    cv::Mat tmp;
    cv::Mat tmp_channel[3];
    switch (cur_channel)
    {
    case H:
    case S:
    case V:
        cvtColor(inImg, tmp, CV_BGR2HSV_FULL);
        split(tmp, tmp_channel);
        return tmp_channel[cur_channel];
    case L:
    case A:
    case B:
        cvtColor(inImg, tmp, CV_BGR2Lab);
        split(tmp, tmp_channel);
        return tmp_channel[cur_channel - 3];
    }
}
