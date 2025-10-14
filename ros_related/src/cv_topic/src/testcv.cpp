#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main() 
{
    Mat image(480, 640, CV_8UC3, Scalar(0, 0, 0));

    randu(image, Scalar::all(0), Scalar::all(255));

    imshow("Random Image", image);
    waitKey(0);

    return 0;
}