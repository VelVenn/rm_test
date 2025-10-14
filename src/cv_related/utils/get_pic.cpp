// This code is used to get images for the camera calibration process

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    VideoCapture player(0, CAP_V4L2);
    if (!player.isOpened()) {
        cerr << "Could not open the camera" << endl;
        return -1;
    }

    player.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    player.set(CAP_PROP_FRAME_WIDTH, 960);
    player.set(CAP_PROP_FRAME_HEIGHT, 540);
    player.set(CAP_PROP_FPS, 30);

    Mat frame;

    for (int i = 0; i < 50; i++) {
        if (!player.read(frame)) {
            cerr << "Web cam connection lost or record is finished" << endl;
            break;
        }

        string filename = format("../res/calib/%02d.png", i + 1);
        imwrite(filename, frame, { IMWRITE_PNG_COMPRESSION, 0 });

        imshow("Web Cam", frame);

        if (waitKey(300) == 27) {
            break;
        }
    }

    destroyAllWindows();
    player.release();
    return 0;
}