#ifndef RECON_H
#define RECON_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace cv;

class Reconstruction
{
public:
    Reconstruction(Mat &camMatrix, Mat &distCoeffe, int *x, int *y, int *z);
    void CreatPointCloud(std::vector<cv::Vec3f> &cloud, std::vector<cv::Vec3b> &color, int *x, int *y, int *z);
    void drawAxisByProject(Mat &frame, Mat &t_R, Mat &t_t);
    void setPointCloud(Mat &frame, Mat &frame_out, Mat &t_R, Mat &t_t, vector<cv::Vec3f> &objectPoints, vector<Vec3b> &objectColor);
    void drawAxis(Mat &frame, Mat &t_R, Mat &t_t);
    void savePcl(string name);

public:
    Mat camMatrix; // 相机内参
    Mat distCoeffs; // 相机畸变
    std::vector<cv::Vec3f> cloud; // 点云, 用于筛选出属于目标的点
    std::vector<cv::Vec3b> color; // 存储BGR颜色
    std::vector<bool> cloud_point_valid; // 存储改点是否有效
};

#endif // RECON_H