#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

void HSV_calib(const cv::Mat img, int* thres, int mode)
{
    // mode: 0 for red; 1 for green; 2 for blue;
    cv::Mat imgHSV;
    cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);

    cv::namedWindow("Control", cv::WINDOW_AUTOSIZE); // create a window called "Control"
    thres[0] = (mode == 0) ? 156 : ((mode == 1) ? 100 : 35);
    thres[1] = (mode == 0) ? 180 : ((mode == 1) ? 140 : 70);
    thres[2] = (mode == 0) ? 43 : ((mode == 1) ? 90 : 43);
    thres[3] = (mode == 0) ? 255 : ((mode == 1) ? 255 : 255);
    thres[4] = (mode == 0) ? 46 : ((mode == 1) ? 90 : 43);
    thres[5] = (mode == 0) ? 255 : ((mode == 1) ? 255 : 255);
    // Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Control", &thres[0], 179); // Hue (0 - 179)
    cv::createTrackbar("HighH", "Control", &thres[1], 179);
    cv::createTrackbar("LowS", "Control", &thres[2], 255); // Saturation (0 - 255)
    cv::createTrackbar("HighS", "Control", &thres[3], 255);
    cv::createTrackbar("LowV", "Control", &thres[4], 255); // Value (0 - 255)
    cv::createTrackbar("HighV", "Control", &thres[5], 255);
    std::vector<cv::Mat> hsvSplit;
    //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
    cv::split(imgHSV, hsvSplit);

    cv::equalizeHist(hsvSplit[2], hsvSplit[2]);
    cv::merge(hsvSplit, imgHSV);
    cv::Mat imgThresholded;
    while (true) {
        cv::inRange(imgHSV, cv::Scalar(thres[0], thres[2], thres[4]), cv::Scalar(thres[1], thres[3], thres[5]),
            imgThresholded); // Threshold the image

        //开操作 (去除一些噪点)
        cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element);

        //闭操作 (连接一些连通域)
        cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element);

        cv::imshow("Thresholded Image", imgThresholded); // show the thresholded image
        cv::imshow("Original", img); // show the original image

        char key = (char)cv::waitKey(300);
        if (key == 27) {
            cv::destroyWindow("Control");
            break;
        } else
            continue;
    }
}

int main()
{
    Mat image = imread("../res/apple.png");

    if (image.empty()) {
        std::cerr << "Could not read the image" << std::endl;
        return 1;
    }

    Mat blurredImage;
    GaussianBlur(image, blurredImage, Size(5, 5), 0);

    Mat hsvImg;
    cvtColor(blurredImage, hsvImg, COLOR_BGR2HSV);
    // cvtColor(image, hsvImg, COLOR_BGR2HSV);

    Mat redMaskLow, redMaskHigh, yellowMask;
    inRange(hsvImg, Scalar(0, 100, 100), Scalar(10, 255, 255), redMaskLow);
    inRange(hsvImg, Scalar(156, 100, 100), Scalar(180, 255, 255), redMaskHigh);
    inRange(hsvImg, Scalar(11, 180, 100), Scalar(27, 255, 255), yellowMask);

    Mat appleMask = redMaskLow | redMaskHigh | yellowMask;

    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    morphologyEx(appleMask, appleMask, cv::MORPH_OPEN, element);
    morphologyEx(appleMask, appleMask, cv::MORPH_CLOSE, element);

    imshow("Original Image", image);
    imshow("Apple Mask", appleMask);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(appleMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> filteredContours;
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        double perimeter = arcLength(contour, true);

        if (area < 500.0) {
            continue;
        }

        double circularity = 0.0;
        if (perimeter > 0) {
            circularity = 4 * CV_PI * area / (perimeter * perimeter);
        }

        if (circularity > 0.7) {
            auto roi = boundingRect(contour);

            rectangle(image, roi, Scalar(255, 0, 0), 2);

            filteredContours.push_back(contour);
        }
    }

    Mat outputImage = image.clone();
    drawContours(outputImage, filteredContours, -1, Scalar(0, 255, 0), 2);

    imshow("Detected Apples", outputImage);

    waitKey(0);
    destroyAllWindows();
    return 0;
}