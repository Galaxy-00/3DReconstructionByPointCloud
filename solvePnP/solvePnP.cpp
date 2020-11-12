#include "solvePnP.h"

SolvePnP_::SolvePnP_(int w, int h, int d, float sl, float ml, string cameraParamsFile, string detectorParamsFile) : w(w),
                                                                                                                    h(h),
                                                                                                                    d(d),
                                                                                                                    sl(sl),
                                                                                                                    ml(ml)
{
    if (!readCameraParams(cameraParamsFile, camMatrix, distCoeffs))
    {
        cerr << "Invalid camera parameters file" << endl;
        return;
    }

    detectorParamters = cv::aruco::DetectorParameters::create();
    if (!readDetectorParameters(detectorParamsFile, detectorParamters))
    {
        cerr << "Invalid detector parameters file" << endl;
        return;
    }

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(d));
    // create charuco board object
    charucoboard = cv::aruco::CharucoBoard::create(w, h, sl, ml, dictionary);
    board = charucoboard.staticCast<cv::aruco::Board>();
}

bool SolvePnP_::solve(cv::Mat &image, cv::Mat &rvec, cv::Mat &tvec)
{
    vector<cv::Point2f> charucoCorners;
    vector<int> charucoIds;
    vector<cv::Point3f> objectPoints;
    vector<cv::Point2f> imagePoints;

    if (detectorCharuco(image, charucoCorners, charucoIds))
    {
        if (charucoCorners.size() > 4) // 至少需要３个点, 此处设置需要５个点
        {
            getObjectAndImagePoints(charucoIds, charucoCorners, objectPoints, imagePoints);
            solvePnP(objectPoints, imagePoints, camMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            return true;
        }
    }
    return false;
}

// 识别charuco获取角点及其id
bool SolvePnP_::detectorCharuco(cv::Mat &inImage, vector<cv::Point2f> &charucoCorners, vector<int> &charucoIds)
{
    vector<int> markerIds;
    vector<vector<cv::Point2f>> markerCorners, rejectedMarkers;

    // 识别角点获取像素位置以及对应id
    cv::aruco::detectMarkers(inImage, dictionary, markerCorners, markerIds, detectorParamters, rejectedMarkers);
    // 再识别
    cv::aruco::refineDetectedMarkers(inImage, board, markerCorners, markerIds, rejectedMarkers,
                                 camMatrix, distCoeffs);
    int interpolatedCorners = 0;
    if (markerIds.size() > 0)
    {
        // 通过插值获取相机内参和畸变
        interpolatedCorners =
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, inImage, charucoboard,
                                             charucoCorners, charucoIds, camMatrix, distCoeffs);
        return true;
    }
    return false;
}

// 以charucoBoard id=0处建立世界坐标系, 给出对应id点在世界坐标系下的三维坐标objectPoints, 以及在图像上的对应的二维坐标imagePoints
void SolvePnP_::getObjectAndImagePoints(vector<int> &charucoIds, vector<cv::Point2f> &charucoCorners, vector<cv::Point3f> &objectPoints, vector<cv::Point2f> &imagePoints)
{
    for (int i = 0; i < charucoIds.size(); i++)
    {
        int cId = charucoIds.at(i);
        int cX = cId % (w - 1);
        int cY = cId / (w - 1);
        cv::Point3f cPoint(
            cX * sl,
            cY * sl,
            0);
        objectPoints.push_back(cPoint);
    }

    for (int i = 0; i < objectPoints.size(); i++)
    {
        imagePoints.push_back(charucoCorners[i]);
    }
}

bool SolvePnP_::readDetectorParameters(string filename, cv::Ptr<cv::aruco::DetectorParameters> &params)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;

    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

bool SolvePnP_::readCameraParams(string file, cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
{
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}