#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

using namespace cv;
using namespace std;

int    IDX[]     = { 9,  10, 11, 12, 13, 14, 15, 16, 17,
					 18, 19, 37, 38, 39, 40, 41, 49, 50 };
string BASE_PATH = "../res/calib/";
string EXT       = ".png";

string CAMERA_NAME  = "chichony_integrated_camera";
int    FRAME_WIDTH  = 960;
int    FRAME_HEIGHT = 540;

Size BOARD_SIZE = { 10, 7 };  // the size of inner corners, cols * rows

float CHECKER_SIZE = 0.015f;  // the size of a square, in meters
	// will define the physical meaning of the extrinsic parameters

int main()
{
	vector<string> images;

	int          idxSize = sizeof(IDX) / sizeof(IDX[0]);
	stringstream path;

	for (int i = 0; i < idxSize; i++) {
		path << BASE_PATH << (IDX[i] < 10 ? "0" : "") << IDX[i] << EXT;
		images.push_back(path.str());

		path.str("");
		path.clear();
	}

	// 3D points for each chessboard images in real world coordinates
	vector<vector<Point3f>> objPoints;
	vector<vector<Point2f>> imgPoints;  // 2D points for each chessboard images

	// 3D points in world coordinates for a chessboard
	vector<Point3f> objWorld;
	for (int i = 0; i < BOARD_SIZE.height; i++) {
		for (int j = 0; j < BOARD_SIZE.width; j++) {
			objWorld.push_back(Point3f(j * CHECKER_SIZE, i * CHECKER_SIZE, 0));
		}
	}

	Mat             img, gray;
	vector<Point2f> corners;  // 2D points for detected chessboard corners
	bool            patternFound = false;

	for (const string& path : images) {
		img = imread(path);

		if (img.empty()) {
			cerr << "Failed to load " << path << endl;
			continue;
		}

		cout << "Checking for: " << path;

		cvtColor(img, gray, COLOR_BGR2GRAY);
		patternFound = findChessboardCorners(gray, BOARD_SIZE, corners);

		if (patternFound) {
			TermCriteria criteria(
				TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001
			);
			cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), criteria);

			objPoints.push_back(objWorld);
			imgPoints.push_back(corners);

			drawChessboardCorners(img, BOARD_SIZE, corners, patternFound);
			imshow("corners", img);

			cout << " -> Chessboard corners found!" << endl;
		} else {
			cerr << " -> Chessboard corners not found!" << endl;
		}

		waitKey(100);
	}

	Mat cameraMatrix, distCoeffs, rvecs, tvecs;
	calibrateCamera(
		objPoints, imgPoints, BOARD_SIZE, cameraMatrix, distCoeffs, rvecs, tvecs
	);

	cout << "\n"
		 << "<<<<<<<<<<<< Calibration result >>>>>>>>>>>>" << endl;

	cout << "Camera matrix: "
		 << "\n"
		 << cameraMatrix << "\n\n";

	cout << "Distortion coefficients: "
		 << "\n"
		 << distCoeffs << "\n\n";

	cout << "Rotation vectors: "
		 << "\n"
		 << rvecs << "\n\n";

	cout << "Translation vectors: "
		 << "\n"
		 << tvecs << "\n\n";

	cout << "<<<<<<<<<<< End >>>>>>>>>>>>" << endl;

	FileStorage writer("../res/calib.yml", FileStorage::WRITE);
	if (writer.isOpened()) {
		writer << "camera_name" << CAMERA_NAME;
		writer << "image_width" << FRAME_WIDTH;
		writer << "image_height" << FRAME_HEIGHT;
		writer << "camera_matrix" << cameraMatrix;
		writer << "distortion_coefficients" << distCoeffs;
		writer << "rotation_vectors" << rvecs;
		writer << "translation_vectors" << tvecs;

		writer.release();
		cout << "Calibration data saved to '../res/calib.yml'" << endl;
	} else {
		cerr << "Error: Could not open the file 'calib.yml' for writing."
			 << endl;
	}

	waitKey(0);
	destroyAllWindows();

	return 0;
}