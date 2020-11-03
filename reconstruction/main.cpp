#include "recon.h"

namespace
{
    const char *about = "Pose estimation using a ChArUco board";
    const char *keys =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }";
} // namespace

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc <= 7)
    {
        parser.printMessage();
        return 0;
    }

    int squaresX = parser.get<int>("w");
    int squaresY = parser.get<int>("h");
    float squareLength = parser.get<float>("sl");
    float markerLength = parser.get<float>("ml");
    int dictionaryId = parser.get<int>("d");
    string camParamFile = parser.get<string>("c");
    string deteParamFile = parser.get<string>("dp");
    int camId = parser.get<int>("ci");

    SolvePnP_ pnp_solver(squaresX, squaresY, dictionaryId, squareLength, markerLength, camParamFile, deteParamFile);
    Segmentation segmenters(0, 255, Segmentation::H, 1);

    // 目标物体在距离世界坐标原点的范围
    int x[] = {-200, 0}; 
    int y[] = {0, 200};
    int z[] = {0, 100};
    Reconstruction recon(pnp_solver.camMatrix, pnp_solver.distCoeffs, x, y, z);

    // 调参窗口
    cv::namedWindow("set_params", CV_WINDOW_NORMAL);
    cv::createTrackbar("channel_idx", "set_params", &segmenters.cur_channel, 5);
    cv::createTrackbar("thresh_min", "set_params", &segmenters.thresh_min, 255);
    cv::createTrackbar("thresh_max", "set_params", &segmenters.thresh_max, 255);
    cv::createTrackbar("cw", "set_params", &segmenters.cw, 1);

    // 3D可视化显示
    viz::Viz3d window("window");
    viz::WCoordinateSystem world_coor(40.0);
    viz::WPlane plane(cv::Size(200, 200));
    viz::WCloud *cloud_widget = NULL;
    window.showWidget("World", world_coor);
    window.showWidget("plane", plane);

    Mat t_R, t_t, frame;
    cv::VideoCapture cp(camId);
    while (cp.grab())
    {
        cp.retrieve(frame);
        Mat frame_copy = frame.clone();

        // Solve Pnp Part
        bool t_pnp_valid = pnp_solver.solve(frame, t_R, t_t);
        if (!t_pnp_valid)
        {
            cout << "not vaild" << endl;
            continue;
        }
        recon.reprojectPoints(frame, t_R, t_t); // 得到相机当前位姿t_R, t_t
        cv::imshow("image", frame);

        // Segment Part
        cv::Mat t_binary, frame_out;
        segmenters.imageProcess(frame_copy, t_binary, frame_out); // 得到frame_out, 用于分割出目标
        cv::imshow("binary", t_binary);
        cv::imshow("segment", frame_out);

        char key = cv::waitKey(1);
        if (key == 'q')
        {
            break;
        }
        else if (key == 'c')
        {
            std::vector<cv::Vec3f> object_cloud;
            std::vector<cv::Vec3b> object_color;
            recon.setPointCloud(frame, frame_out, t_R, t_t, object_cloud, object_color);

            if (cloud_widget == NULL)
            {
                cloud_widget = new viz::WCloud(object_cloud, object_color);
            }
            else
            {
                delete cloud_widget;
                cloud_widget = new viz::WCloud(object_cloud, object_color);
            }
        }

        if (cloud_widget != NULL)
        {
            window.showWidget("point_cloud", *cloud_widget);
        }
        window.spinOnce(10, false);
    }
}