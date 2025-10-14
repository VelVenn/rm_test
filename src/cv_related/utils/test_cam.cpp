#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture player(0, CAP_V4L2);
    if (!player.isOpened()) {
        cerr << "Could not open the camera" << endl;
        return -1;
    }

    player.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    player.set(CAP_PROP_FRAME_WIDTH, 960);
    player.set(CAP_PROP_FRAME_HEIGHT, 540);

    Mat frame;
    while (true) {
        if (!player.read(frame)) {
            cerr << "Web cam connection lost or record is finished" << endl;
            break;
        }

        imshow("Web Cam", frame);
        if (waitKey(1000/30) >= 0) 
            break;
    }

    destroyAllWindows();
    return 0;
}