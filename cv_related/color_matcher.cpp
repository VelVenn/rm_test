#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
    // 1. 读取图像
    Mat image = imread("../res/ColorBlock.png");

    if (image.empty()) {
        cout << "Could not open or find the image" << endl;
        return -1;
    }

    imshow("Original Image", image);

    // 2. 颜色空间转换 (BGR to HSV)
    Mat hsv_image;
    cvtColor(image, hsv_image, COLOR_BGR2HSV);

    // 3. 颜色分割 (鲜红色)
    // 定义鲜红色的 HSV 范围 (更窄的色调范围，更高的饱和度和亮度)
    Scalar lower_red = Scalar(0, 150, 150); // 鲜红色低阈值 (H:0-10, S:150-255, V:150-255)
    Scalar upper_red = Scalar(10, 255, 255); // 鲜红色高阈值
    Mat red_mask1;
    inRange(hsv_image, lower_red, upper_red, red_mask1);

    Scalar lower_red2 = Scalar(160, 150, 150); // 鲜红色低阈值 (H:160-180, S:150-255, V:150-255)
    Scalar upper_red2 = Scalar(180, 255, 255); // 鲜红色高阈值
    Mat red_mask2;
    inRange(hsv_image, lower_red2, upper_red2, red_mask2);

    Mat red_mask = red_mask1 | red_mask2; // 合并两个红色范围的掩码

    // 4. 边缘检测 (Canny)
    Mat canny_output;
    Canny(red_mask, canny_output, 100, 200); // 100 和 200 是 Canny 阈值

    // 5. 轮廓提取
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(canny_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 6. 面积过滤
    vector<vector<Point>> filtered_contours;
    double min_area = 50.0; // 最小面积阈值
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > min_area) {
            filtered_contours.push_back(contours[i]);
        }
    }

    // 7. 在原始图像上绘制轮廓
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    for (size_t i = 0; i < filtered_contours.size(); i++) {
        Scalar color = Scalar(0, 0, 0); 
        drawContours(image, filtered_contours, (int)i, color, 2, LINE_8, hierarchy, 0);
    }

    // 8. 显示结果
    imshow("Red Mask", red_mask);
    imshow("Canny Output", canny_output);
    imshow("Contours", image);

    imwrite("../res/RedContours.png", image);

    waitKey(0);
    return 0;
}
