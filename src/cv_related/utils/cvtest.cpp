#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
    Mat test = Mat::zeros(800, 800, CV_8UC3);
    
    while(1) 
    {
        randu(test, 0, 255);

        imshow("test", test);
        if(waitKey(1000) >= 0) break;
    }

    return 0;
}
