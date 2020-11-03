#ifndef RECON_H
#define RECON_H

#include "../segmentation/segmentation.h"
#include "../solvePnP/solvePnP.h"

class Reconstruction
{
public:
    Reconstruction(Mat &camMatrix, Mat &distCoeffe, int *x, int *y, int *z);
    void CreatPointCloud(std::vector<cv::Vec3f> &cloud, std::vector<cv::Vec3b> &color, int *x, int *y, int *z);
    void drawAxisByProject(Mat &frame, Mat &t_R, Mat &t_t);
    void setPointCloud(Mat &frame, Mat &frame_out, Mat &t_R, Mat &t_t, vector<cv::Vec3f> &objectPoints, vector<Vec3b> &objectColor);
    void drawAxis(Mat &frame, Mat &t_R, Mat &t_t);

public:
    Mat camMatrix;
    Mat distCoeffs;
    std::vector<cv::Vec3f> cloud; // 存储x,y,z位置
    std::vector<cv::Vec3b> color; // 存储BGR颜色
    std::vector<bool> cloud_point_valid;
    std::vector<cv::Point2f> reproj_point;
};

#endif // RECON_H