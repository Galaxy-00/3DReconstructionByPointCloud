#include "./segmentation.h"

const char *keys =
    "{ci       | 0     | Camera id if input doesnt come from video (-v) }";

int main(int argc, char *argv[])
{
    // 解析命令行参数
    cv::CommandLineParser parser(argc, argv, keys);
    int cam_id = parser.get<int>("ci");
    cv::VideoCapture cp(cam_id);

    Segmentation test_vision(0, 255, Segmentation::H, 1);
    cv::namedWindow("set_params", CV_WINDOW_NORMAL);
    cv::createTrackbar("channel_idx", "set_params", &test_vision.cur_channel, 5);
    cv::createTrackbar("thresh_min", "set_params", &test_vision.thresh_min, 255);
    cv::createTrackbar("thresh_max", "set_params", &test_vision.thresh_max, 255);
    cv::createTrackbar("cw", "set_params", &test_vision.cw, 1);

    while (cp.grab())
    {
        cv::Mat frame, binary_frame, frame_out;
        cp.retrieve(frame);

        test_vision.imageProcess(frame, binary_frame, frame_out);

        cv::imshow("frame", frame);
        cv::imshow("binary", binary_frame);
        cv::imshow("result", frame_out);
        char key = cv::waitKey(1);
        if (key == 'q')
        {
            break;
        }
    }

    return 0;
}
