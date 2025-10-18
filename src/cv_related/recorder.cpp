#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace cv;
using namespace std;

int main()
{
	VideoCapture recorder(0, CAP_V4L2);

	if (!recorder.isOpened()) {
		cerr << "Could not open the camera" << endl;
		return -1;
	}

	recorder.set(CAP_PROP_FRAME_WIDTH, 640);
	recorder.set(CAP_PROP_FRAME_HEIGHT, 480);
	recorder.set(
		CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')
	);  // Need to set the codec explicitly in WSL to avoid errors

	VideoWriter video(
		"../res/test.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0,
		Size(640, 480)
	);

	if (!video.isOpened()) {
		cerr << "Could not open the video file for writing" << endl;
		return -1;
	}

	Mat          frame;
	stringstream fsize, fps;  // Frame size and FPS

	// int BRI = recorder.get(CAP_PROP_BRIGHTNESS);
	namedWindow("Brightness", WINDOW_NORMAL);

	// createTrackbar("Brightness", "Brightness", &BRI, 255);

	createTrackbar(
		"Brightness", "Brightness", nullptr, 255,
		[](int pos, void* userdata) {
			VideoCapture* self = static_cast<VideoCapture*>(userdata
			);  // Cast the userdata to the class type
			self->set(CAP_PROP_BRIGHTNESS, pos);
		},
		&recorder
	);

	while (true) {
		if (!recorder.read(frame)) {
			cerr << "Web cam connection lost or record is finished" << endl;
			break;
		}

		// recorder.set(CAP_PROP_BRIGHTNESS, BRI);

		// resize(frame, resizeFrame, Size(640, 480));
		// Need to resize the frame before writing to the video
		// or the video can't be rendered correctly

		video.write(frame);

		fsize << "Frame Size: " << frame.size().width << " * "
			  << frame.size().height;
		fps << "FPS: " << recorder.get(CAP_PROP_FPS);

		putText(
			frame, fsize.str(), Point(10, 10), FONT_HERSHEY_COMPLEX, 0.4,
			Scalar(0, 0, 255), 1
		);
		putText(
			frame, fps.str(), Point(10, 20), FONT_HERSHEY_COMPLEX, 0.4,
			Scalar(0, 0, 255), 1
		);

		imshow("Web Cam", frame);

		fsize.str("");
		fps.str("");

		fsize.clear();
		fps.clear();

		if (waitKey(1000 / 30) == 27) { break; }
	}

	recorder.release();
	video.release();
	destroyAllWindows();

	return 0;
}
