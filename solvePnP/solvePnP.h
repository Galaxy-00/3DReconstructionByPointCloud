#ifndef SOLVEPNP_H
#define SOLVEPNP_H

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>
#include <fstream>
#include <opencv2/highgui.hpp>

using namespace std;

class SolvePnP_
{
public:
    // 构造函数
    SolvePnP_(int w, int h, int d, float sl, float ml, string cameraParamsFile, string detectorParamsFile);
    // solvePnP主要过程, 通过世界坐标和像素坐标得出当前相机的旋转矩阵和平移向量
    bool solve(cv::Mat &image, cv::Mat &rvecs, cv::Mat &tvecs);
    // 读取相机内参和相机畸变
    static bool readCameraParams(string file, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
    // 读取识别参数
    static bool readDetectorParameters(string filename, cv::Ptr<cv::aruco::DetectorParameters> &detectorParamterss);

public:
    int w;                                            // 所用charuco板的width, 即宽上格子的数量
    int h;                                            // 所用charuco板的height
    int d;                                            // 所用charuco板的dictionary
    float sl;                                         // square_length 一个格子的长宽 单位: mm
    float ml;                                         // marker_length 一个marker的长宽 单位: mm
    cv::Mat camMatrix;                                    // 相机内参矩阵
    cv::Mat distCoeffs;                                   // 相机畸变矩阵
    cv::Ptr<cv::aruco::DetectorParameters> detectorParamters; // 视觉识别参数, 具体文档在opencv charuco/tutorial/aruco_detection中
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard;            // charuco board对象指针
    cv::Ptr<cv::aruco::Board> board;                          // board 对象指针
    cv::Ptr<cv::aruco::Dictionary> dictionary;                // 创建board所用的预定义集

    // 识别inImage, 得出charuco中角的像素坐标以及对应的id
    bool detectorCharuco(cv::Mat &inImage, vector<cv::Point2f> &charucoCorners, vector<int> &charucoIds);
    // 通过id和corner的坐标得出 世界坐标系中对应的坐标以及像素坐标, 用于solvePnP
    void getObjectAndImagePoints(vector<int> &charucoIds, vector<cv::Point2f> &charucoCorners, vector<cv::Point3f> &objectPoints, vector<cv::Point2f> &imagePoints); // 得到世界坐标下的坐标
};

#endif // SLOVEPNP_H