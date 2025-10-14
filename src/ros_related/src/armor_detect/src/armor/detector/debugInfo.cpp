#include "armor_detect/armor/detector.h"

#include <opencv2/opencv.hpp>
#include <string>

#include "armor_detect/armor/armorPad.h"
#include "armor_detect/armor/lightBar.h"

#include "armor_detect/utils/timer.h"

using namespace cv;
using namespace std;

namespace armor {

bool haveSomeDebugWindowClosed(unsigned flags)
{
	if (flags == SILENCE) { return false; }

	static bool frameCreated     = false;
	static bool binaryCreated    = false;
	static bool armorInfoCreated = false;

	if (flags & SHOW_FRAME) {
		double prop = getWindowProperty("Frame", WND_PROP_VISIBLE);
		if (prop >= 0) { frameCreated = true; }
		if (frameCreated && prop < 1) { return true; }
	}

	if (flags & SHOW_BINARY) {
		double prop = getWindowProperty("Binary", WND_PROP_VISIBLE);
		if (prop >= 0) { binaryCreated = true; }
		if (binaryCreated && prop < 1) { return true; }
	}

	if (flags & ALL_ARMOR_INFO) {
		double prop = getWindowProperty("Armor Info", WND_PROP_VISIBLE);
		if (prop >= 0) { armorInfoCreated = true; }
		if (armorInfoCreated && prop < 1) { return true; }
	}

	return false;
}

static void showBinary(unsigned flags, const Mat& bi)
{
	if (!(flags & SHOW_BINARY)) { return; }

	namedWindow("Binary", WINDOW_NORMAL);

	Mat display;
	bi.copyTo(display);

	imshow("Binary", display);
}

static void drawLights(
	unsigned flags, DetectorState state, const vector<LightBar>& lights,
	const Mat& src
)
{
	if (!(flags & SHOW_LIGHTS)) { return; }

	if (state < LIGHTS_FOUND || state >= UNKNOWN_DETECTOR_STATE) { return; }

	for (const auto& light : lights) {
		Point2f pts[4];
		light.getBox().points(pts);
		for (int j = 0; j < 4; j++) {
			line(src, pts[j], pts[(j + 1) % 4], Scalar(255, 0, 255), 2);
		}

		circle(src, light.getCenter(), 4, Scalar(43, 139, 50), -1);
	}
}

static void drawArmors(
	unsigned flags, DetectorState state, const vector<ArmorPad>& armors,
	const Mat& src
)
{
	if (!(flags & SHOW_ARMORS) || !(flags & ALL_ARMOR_INFO)) { return; }

	if (state < ARMORS_FOUND || state >= UNKNOWN_DETECTOR_STATE) { return; }

	for (const auto& armor : armors) {
		auto vertices = armor.getVertices();
		for (int i = 0; i < 4; ++i) {
			line(src, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2);
		}

		circle(src, armor.getCenter(), 4, Scalar(0, 255, 255), -1);
	}
}

static void drawTimeStats(unsigned flags, const TimeStats& stats, Mat& src)
{
	if (!(flags & SHOW_FPS)) { return; }

	string fpsText  = "FPS: " + to_string(stats.fps);
	string procText = "Cur Proc: " + to_string(stats.lastProcessTime) + " ms";
	string avgProc  = "Avg Proc: " + to_string(stats.avgProcessTime) + " ms";

	putText(
		src, fpsText, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.5,
		Scalar(0, 255, 0), 2
	);
	putText(
		src, procText, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.5,
		Scalar(0, 255, 0), 2
	);
	putText(
		src, avgProc, Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.5,
		Scalar(0, 255, 0), 2
	);
}

static void drawArmorNumAndSave(
	unsigned flags, DetectorState state, const vector<ArmorPad>& armors,
	Mat& src
)
{
	static long FILE_COUNT = 0;

	if (!(flags & SHOW_ARMOR_NUM)) { return; }

	if (state < ARMORS_FOUND || state >= UNKNOWN_DETECTOR_STATE) { return; }

	for (size_t i = 0; i < armors.size(); ++i) {
		auto armor = armors[i];

		string numText =
			armor.getNumber() == -1 ? "N/A" : to_string(armor.getNumber());

		Point2f pos = armor.getCenter() + Point2f(10, -10);

		putText(
			src, numText, pos, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(240, 90, 170),
			2
		);

		if (flags & SAVE_NUM_IMG) {
			string base = "./detected_armor/";
			string num  = armor.getNumber() == -1
							  ? "Unknown/"
							  : to_string(armor.getNumber()) + "/";
			string dir  = base + num;

// Create directory if it doesn't exist
#if defined(_WIN32)
			_mkdir(dir.c_str());
#else
			system(("mkdir -p " + dir).c_str());
#endif

			// Save the image
			string filename =
				dir + to_string(i) + "-" + to_string(FILE_COUNT++) + ".png";
			imwrite(filename, armor.getImage());
		}
	}
}

static void drawArmorPos(
	unsigned flags, DetectorState state, const vector<ArmorPad>& armors,
	Mat& src
)
{
	if (!(flags & SHOW_ARMOR_POS)) { return; }

	if (state < ARMORS_FOUND || state >= UNKNOWN_DETECTOR_STATE) { return; }

	for (size_t i = 0; i < armors.size(); ++i) {
		auto armor = armors[i];

		string posX = format("%.2f", armor.getCenter().x).c_str();
		string posY = format("%.2f", armor.getCenter().y).c_str();

		putText(
			src, "X:" + posX, armor.getCenter() + Point2f(10, 10),
			FONT_HERSHEY_SIMPLEX, 0.4, Scalar(240, 90, 170), 1
		);
		putText(
			src, "Y:" + posY, armor.getCenter() + Point2f(10, 30),
			FONT_HERSHEY_SIMPLEX, 0.4, Scalar(240, 90, 170), 1
		);
	}
}

static void showFrame(unsigned flags, const Detector& instance)
{
	if (!(flags & SHOW_FRAME)) { return; }

	namedWindow("Frame", WINDOW_NORMAL);

	Mat display = instance.getFrame().clone();

	drawLights(flags, instance.getState(), instance.getLights(), display);
	drawArmors(flags, instance.getState(), instance.getArmors(), display);
	drawTimeStats(flags, instance.getTimeStats(), display);

	imshow("Frame", display);
}

static void showArmorInfo(unsigned flags, const Detector& instance)
{
	auto state = instance.getState();

	if (!(flags & ALL_ARMOR_INFO)) { return; }

	namedWindow("Armor Info", WINDOW_NORMAL);

	Mat display = instance.getFrame().clone();

	drawArmors(flags, state, instance.getArmors(), display);

	if (!instance.getNumClassifier().isModelLoaded()) {
		putText(
			display, "NO MODEL LOADED", Point(10, 30), FONT_HERSHEY_SIMPLEX,
			0.6, Scalar(0, 0, 255), 2
		);
	} else {
		drawArmorNumAndSave(flags, state, instance.getArmors(), display);
	}

	drawArmorPos(flags, state, instance.getArmors(), display);

	imshow("Armor Info", display);
}

void Detector::showDebugInfo(unsigned flags) const
{
	if (flags == SILENCE) { return; }

	showFrame(flags, *this);
	showArmorInfo(flags, *this);
	showBinary(flags, binary);
}

}  // namespace armor