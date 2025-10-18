#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
	Mat image = imread("../res/ColorBlock.png");

	if (image.empty()) {
		cout << "Could not open or find the image" << endl;
		return -1;
	}

	imshow("Original Image", image);

	Mat hsv_image;
	cvtColor(image, hsv_image, COLOR_BGR2HSV);

	Scalar lower_red = Scalar(0, 150, 150);
	Scalar upper_red = Scalar(10, 255, 255);
	Mat    red_lo;
	inRange(hsv_image, lower_red, upper_red, red_lo);

	Scalar lower_red2 = Scalar(160, 150, 150);
	Scalar upper_red2 = Scalar(180, 255, 255);
	Mat    red_hi;
	inRange(hsv_image, lower_red2, upper_red2, red_hi);

	Mat red_mask = red_lo | red_hi;

	Mat canny_output;
	Canny(red_mask, canny_output, 100, 200);

	vector<vector<Point>> contours;
	vector<Vec4i>         hierarchy;
	findContours(
		canny_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE
	);

	vector<vector<Point>> filtered_contours;
	double                min_area = 50.0;
	for (size_t i = 0; i < contours.size(); i++) {
		double area = contourArea(contours[i]);
		if (area > min_area) { filtered_contours.push_back(contours[i]); }
	}

	Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
	for (size_t i = 0; i < filtered_contours.size(); i++) {
		Scalar color = Scalar(0, 0, 0);
		drawContours(
			image, filtered_contours, (int)i, color, 2, LINE_8, hierarchy, 0
		);
	}

	imshow("Red Mask", red_mask);
	imshow("Canny Output", canny_output);
	imshow("Contours", image);

	imwrite("../res/RedContours.png", image);

	waitKey(0);
	return 0;
}
