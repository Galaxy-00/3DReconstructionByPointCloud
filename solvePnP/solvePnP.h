#ifndef SOLVEPNP_H
#define SLOVEPNP_H

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>

using namespace cv;
using namespace std;

class SolvePnP_
{
public:
    SolvePnP_(int w, int h, int d, int sl, int ml, string cameraParamsFile, string detectorParamsFile);
    bool solve(Mat &image, Mat &Rvecs, Mat &Tvecs);
    static bool readCameraParams(String file, Mat &cameraMatrix, Mat &distCoeffs);
    static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params);

private:
    int w;          // 所用charuco板的width, 即宽上格子的数量
    int h;          // 所用charuco板的height
    int d;          // 所用charuco板的dictionary
    int sl;         // square_length 一个格子的长宽 单位: mm
    int ml;         // marker_length 一个marker的长宽 单位: mm
    Mat camMatrix;  // 相机内参矩阵
    Mat distCoeffs; // 相机畸变矩阵
    Ptr<aruco::DetectorParameters> detectorParams;
    Ptr<aruco::CharucoBoard> charucoboard;
    Ptr<aruco::Board> board;
    Ptr<aruco::Dictionary> dictionary;

    bool detectorCharuco(Mat &inImage, vector<Point2f> &charucoCorners, vector<int> &charucoIds);
    void getObjectAndImagePoints(vector<int> &charucoIds, vector<Point2f> &charucoCorners, vector<Point3f> &objectPoints, vector<Point2f> &imagePoints); // 得到世界坐标下的坐标
};

#endif // SLOVEPNP_H