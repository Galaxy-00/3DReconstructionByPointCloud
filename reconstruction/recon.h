#ifndef RECON_H
#define RECON_H

#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> // pcl和opencv在flann上有冲突, pcl应放在opencv include前, 不使用namespace cv
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

using namespace std;

class Reconstruction
{
public:
    Reconstruction(cv::Mat &camMatrix, cv::Mat &distCoeffe, int *x, int *y, int *z);
    void CreatPointCloud(std::vector<cv::Vec3f> &cloud, std::vector<cv::Vec3b> &color, int *x, int *y, int *z);
    void drawAxisByProject(cv::Mat &frame, cv::Mat &t_R, cv::Mat &t_t);
    void setPointCloud(cv::Mat &frame, cv::Mat &frame_out, cv::Mat &t_R, cv::Mat &t_t, vector<cv::Vec3f> &objectPoints, vector<cv::Vec3b> &objectColor);
    void drawAxis(cv::Mat &frame, cv::Mat &t_R, cv::Mat &t_t);
    void savePcl(string name);

public:
    cv::Mat camMatrix; // 相机内参
    cv::Mat distCoeffs; // 相机畸变
    std::vector<cv::Vec3f> cloud; // 点云, 用于筛选出属于目标的点
    std::vector<cv::Vec3b> color; // 存储BGR颜色
    std::vector<bool> cloud_point_valid; // 存储改点是否有效
};

#endif // RECON_H