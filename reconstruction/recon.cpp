#include "recon.h"

Reconstruction::Reconstruction(cv::Mat &camMatrix, cv::Mat &distCoeffs, int *x, int *y, int *z)
{
    this->camMatrix = camMatrix;
    this->distCoeffs = distCoeffs;

    // 重建时候用到的点云
    CreatPointCloud(cloud, color, x, y, z);
    for (int i = 0; i < cloud.size(); i++)
    {
        cloud_point_valid.push_back(true);
    }
}

void Reconstruction::drawAxisByProject(cv::Mat &frame, cv::Mat &t_R, cv::Mat &t_t)
{
    std::vector<cv::Point2f> reproj_point; // 重投影二维点
    // 利用t_R和t_t 重投影, 画出三维坐标轴
    std::vector<cv::Point3f> ori_pnt;
    ori_pnt.push_back(cv::Point3f(0, 0, 0)); // 坐标原点
    ori_pnt.push_back(cv::Point3f(0, 60, 0));
    ori_pnt.push_back(cv::Point3f(60, 0, 0));
    ori_pnt.push_back(cv::Point3f(0, 0, 60));
    // 将ori_pnt 投影到二维平面图像上
    cv::projectPoints(ori_pnt, t_R, t_t, camMatrix, distCoeffs, reproj_point);

    cv::circle(frame, reproj_point[0], 5, cv::Scalar(0, 255, 255), 3);
    cv::circle(frame, reproj_point[1], 5, cv::Scalar(0, 0, 0), 3);
    cv::circle(frame, reproj_point[2], 5, cv::Scalar(0, 255, 0), 3);
    cv::circle(frame, reproj_point[3], 5, cv::Scalar(255, 255, 0), 3);
    cv::line(frame, reproj_point[0], reproj_point[1], cv::Scalar(0, 255, 0), 3); // y轴, 绿色
    cv::line(frame, reproj_point[0], reproj_point[2], cv::Scalar(0, 0, 255), 3); // x轴, 红色
    cv::line(frame, reproj_point[0], reproj_point[3], cv::Scalar(255, 0, 0), 3); // z轴, 蓝色
}

void Reconstruction::drawAxis(cv::Mat &frame, cv::Mat &t_R, cv::Mat &t_t)
{
    float axisLength = 3 * 0.34;
    cv::aruco::drawAxis(frame, camMatrix, distCoeffs, t_R, t_t, axisLength);
}

void Reconstruction::setPointCloud(cv::Mat &frame, cv::Mat &frame_out, cv::Mat &t_R, cv::Mat &t_t, vector<cv::Vec3f> &object_cloud, vector<cv::Vec3b> &object_color)
{
    cv::Mat frame_copy = frame.clone();
    std::vector<cv::Point2f> reproj_point; // 重投影二维点

    // 函数cvProjectPoints2通过给定的内参数和外参数计算三维点投影到二维图像平面上的坐标
    // 将cloud范围内的点重投影到二维平面上, 并通过阈值分割后的图像来提取含有目标的点
    cv::projectPoints(cloud, t_R, t_t, camMatrix, distCoeffs, reproj_point);
    for (int j = 0; j < reproj_point.size(); j++)
    {
        int t_y = (int)reproj_point[j].y;
        int t_x = (int)reproj_point[j].x;
        if (t_x < 0 || t_x >= frame.cols || t_y < 0 || t_y >= frame.rows)
        {
            continue;
        }
        if (frame_out.at<uchar>(t_y, t_x) == 255) // 筛选出无效的点
        {
            cloud_point_valid[j] = false;
        }
        // else if (frame_out.at<uchar>(t_y, t_x) == 0) // 重置为有效
        // {
        // cloud_point_valid[j] = true;
        // }
    }

    // viz part
    // camera_dists[i][0] 是i点和相机的距离， camera_dists[i][1] 是对应点i云中的idx, 距离初始化为99999999, index初始化为-1
    cv::Mat camera_dists(frame_copy.rows, frame_copy.cols, CV_64FC2, cv::Scalar(99999999, -1));

    std::vector<cv::Vec2i> object_cloud_reproj;
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud_point_valid[i])
        {
            object_cloud.push_back(cloud[i]); // 添加有效的三维点
            object_color.push_back(color[i]);
            cv::Point2i t_pnt = reproj_point[i]; // 对应云i点在二维平面上的坐标, 二维点
            object_cloud_reproj.push_back(t_pnt);

            // 计算体素与相机的距离
            double t_dist = sqrt(
                (cloud[i][0] - t_t.at<float>(0, 0)) * (cloud[i][0] - t_t.at<float>(0, 0)) +
                (cloud[i][1] - t_t.at<float>(0, 1)) * (cloud[i][1] - t_t.at<float>(0, 1)) +
                (cloud[i][2] - t_t.at<float>(0, 2)) * (cloud[i][2] - t_t.at<float>(0, 2)));

            if (t_pnt.x < 0 || t_pnt.x >= camera_dists.cols || t_pnt.y < 0 || t_pnt.y >= camera_dists.rows)
            {
                continue;
            }
            if (t_dist < camera_dists.at<cv::Vec2d>(t_pnt)[0]) // 小于当前camera_dists中的值
            {
                camera_dists.at<cv::Vec2d>(t_pnt)[0] = t_dist;
                camera_dists.at<cv::Vec2d>(t_pnt)[1] = i;
            }
        }
    }

    // 设置有效点的颜色, 进一步筛选
    for (int i = 0; i < object_cloud.size(); i++)
    {
        cv::Point2i t_reproj = object_cloud_reproj[i];
        if (t_reproj.x < 0 || t_reproj.x >= camera_dists.cols || t_reproj.y < 0 || t_reproj.y >= camera_dists.rows)
        {
            continue;
        }
        if (camera_dists.at<cv::Vec2d>(t_reproj)[1] != -1)
        {
            object_color[i] = frame_copy.at<cv::Vec3b>(t_reproj);
            color[i] = object_color[i];
        }
    }

    if (object_cloud.size() == 0)
    {
        object_cloud.push_back(cloud[0]);
        object_color.push_back(color[0]);
    }
}

// 初始化点云
void Reconstruction::CreatPointCloud(std::vector<cv::Vec3f> &cloud, std::vector<cv::Vec3b> &color, int *x, int *y, int *z)
{
    for (int tx = x[0]; tx <= x[1]; tx++)
    {
        for (int ty = y[0]; ty <= y[1]; ty++)
        {
            for (int tz = z[0]; tz <= z[1]; tz++)
            {
                cloud.push_back(
                    cv::Vec3f(tx, ty, tz));
                color.push_back(
                    cv::Vec3b(0, 0, 0));
            }
        }
    }
}

// 通过pcl保存点云, 以便展示
void Reconstruction::savePcl(string name)
{
    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud_point_valid.at(i))
        {
            PointT p;
            p.x = cloud.at(i)[0];
            p.y = cloud.at(i)[1];
            p.z = cloud.at(i)[2];
            p.r = color.at(i)[2];
            p.g = color.at(i)[1];
            p.b = color.at(i)[0];
            pointCloud->points.push_back(p);
        }
    }

    // depth filter and statistical removal
    // pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    // statistical_filter.setMeanK(50);
    // statistical_filter.setStddevMulThresh(1.0);
    // statistical_filter.setInputCloud(pointCloud);
    // statistical_filter.filter(*pointCloud);
    // pointCloud->is_dense = false;

    cout << "点云共有" << pointCloud->size() << "个点." << endl;

    // voxel filter
    // pcl::VoxelGrid<PointT> voxel_filter;
    // double resolution = 0.03;
    // voxel_filter.setLeafSize(resolution, resolution, resolution); // resolution
    // PointCloud::Ptr tmp(new PointCloud);
    // voxel_filter.setInputCloud(pointCloud);
    // voxel_filter.filter(*pointCloud);
    // cout << "滤波之后，点云共有" << pointCloud->size() << "个点." << endl;

    pcl::io::savePCDFileBinary(name, *pointCloud);
}