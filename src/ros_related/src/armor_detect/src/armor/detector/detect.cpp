#include "armor_detect/armor/detector.h"

#include <functional>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

#include "armor_detect/armor/armorPad.h"
#include "armor_detect/armor/armorParam.h"
#include "armor_detect/armor/lightBar.h"

#include "armor_detect/utils/timer.h"
#include "armor_detect/utils/utils.h"

using namespace cv;
using namespace std;

namespace armor {

Detector::Detector(ArmorColor _color, NumClassifier _classifier) :
	classifier(_classifier),
	enemyColor(_color)
{
	lutTable       = Mat(1, 256, CV_8UC1);
	uchar* lutData = lutTable.data;

	for (int i = 0; i < 256; i++) {
		lutData[i] = saturate_cast<uchar>(
			pow(i / 255.0f, preDefCameraParam.hiGamma) * 255.0f
		);
	}
}

void Detector::reset()
{
	state = UNDETECTED;
	lights.clear();
	armors.clear();
}

bool Detector::preprocess(Mat& src)
{
	if (enemyColor >= UNKNOWN_ARMOR_COLOR) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Enemy color not set!" << endl;
		}

		return false;
	}

	if (src.empty()) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Input frame is empty!" << endl;
		}

		return false;
	}

	binary = Mat::zeros(src.size(), CV_8UC1);

	// LUT(src, lutTable, src);

	uchar* srcData = src.data;
	uchar* biData  = binary.data;

	auto redPreprocess = [&srcData, &biData](const Range& range) {
		for (int i = range.start; i < range.end; i++) {
			if (srcData[i * 3 + 2] - srcData[i * 3] >
					preDefArmorParam.colorThreshold &&
				srcData[i * 3 + 2] > preDefArmorParam.brightThreshold) {
				biData[i] = 255;

				continue;
			}

			// if (srcData[i * 3] > preDefArmorParam.brightThreshold &&
			// 	srcData[i * 3 + 1] > preDefArmorParam.brightThreshold &&
			// 	srcData[i * 3 + 2] > preDefArmorParam.brightThreshold &&
			// 	srcData[i * 3 + 2] > srcData[i * 3 + 1] &&
			// 	srcData[i * 3 + 2] > srcData[i * 3]) {
			// 	biData[i] = 255;
			// }
		}
	};

	auto bluePreprocess = [&srcData, &biData](const Range& range) {
		for (int i = range.start; i < range.end; i++) {
			if (srcData[i * 3] - srcData[i * 3 + 2] >
					preDefArmorParam.colorThreshold &&
				srcData[i * 3] > preDefArmorParam.brightThreshold) {
				biData[i] = 255;

				continue;
			}

			// if (srcData[i * 3] > preDefArmorParam.brightThreshold &&
			// 	srcData[i * 3 + 1] > preDefArmorParam.brightThreshold &&
			// 	srcData[i * 3 + 2] > preDefArmorParam.brightThreshold &&
			// 	srcData[i * 3] > srcData[i * 3 + 1]) {
			// 	biData[i] = 255;
			// }
		}
	};

	auto preproccessor = (enemyColor == RED_ARMOR)
							 ? function<void(const Range&)>(redPreprocess)
							 : function<void(const Range&)>(bluePreprocess);

	// ref 1: https://docs.opencv.org/4.11.0/dc/ddf/tutorial_how_to_use_OpenCV_parallel_for_new.html
	// ref 2: https://blog.csdn.net/qq_28087491/article/details/118992396
	parallel_for_(Range(0, src.rows * src.cols), preproccessor);

	auto dElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	auto eElement = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));

	dilate(binary, binary, dElement);
	// erode(binary, binary, eElement);

	return true;
}

bool Detector::detectLights()
{
	if (binary.empty() || enemyColor >= UNKNOWN_ARMOR_COLOR) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Frame is empty or enemy color not set!" << endl;
		}

		return false;
	}

	vector<vector<Point>> contours;
	findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	mutex lightMutex;

	parallel_for_(
		Range(0, contours.size()),
		[this, &contours, &lightMutex](const Range& range) {
			for (int i = range.start; i < range.end; i++) {
				const auto& contour = contours[i];

				if (contour.size() < 5) { continue; }

				RotatedRect box = fitEllipse(contour);

				double area    = contourArea(contour);
				double boxArea = box.size.width * box.size.height;

				if (area < 1e-6 || boxArea < 1e-6) { continue; }

				float solidity = area / boxArea;

				// cout << "solidity: " << solidity << endl;

				if (solidity < preDefArmorParam.minLightSolidity) { continue; }

				LightBar light(box);

				if (light.isValid()) {
					lock_guard<mutex> lock(lightMutex);
					this->lights.push_back(light);
				}
			}
		}
	);

	if (lights.size() < 2) {
		if (debugLevel >= INFO) {
			cout << "[INFO] Not enough lights detected: " << lights.size()
				 << endl;
		}

		state = UNDETECTED;
		return false;
	}

	state = LIGHTS_FOUND;

	sort(
		lights.begin(), lights.end(),
		[](const LightBar& a, const LightBar& b) {
			return a.getCenter().x < b.getCenter().x;
		}
	);

	return true;
}

bool Detector::detectArmors()
{
	if (state < LIGHTS_FOUND) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Lights not detected!" << endl;
		}

		return false;
	}

	for (size_t i = 0; i < lights.size() - 1; i++) {
		for (size_t j = i + 1; j < lights.size(); j++) {
			ArmorPad armor(lights[i], lights[j]);

			if (armor.getLightsXDiffRatio() >
				preDefArmorParam.maxLightsXDiffRatio) {
				break;
			}

			if (armor.isSuitable()) {
				armor.setLeftIdx(i);
				armor.setRightIdx(j);

				armor.setNumber(classifier.classify(frame, armor));

				armors.push_back(std::move(armor));
			}
		}
	}

	removeRepeatArmors();

	if (armors.empty()) {
		if (debugLevel >= INFO) {
			cout << "[INFO] No armors detected!" << endl;
		}

		state = LIGHTS_FOUND;
		return false;
	}

	state = ARMORS_FOUND;
	return true;
}

bool Detector::removeRepeatArmors()
{
	if (armors.empty()) { return false; }

	// 记录每个灯条当前对应的最佳装甲板的索引
	unordered_map<int, size_t> betterLeft, betterRight;
	vector<bool>               keep(armors.size(), true);

	for (size_t i = 0; i < armors.size(); i++) {
		int   leftIdx  = armors[i].getLeftIdx();
		int   rightIdx = armors[i].getRightIdx();
		float dAngle   = armors[i].getLightsDeviationAngle();

		if (betterLeft.count(leftIdx)) {
			size_t j = betterLeft[leftIdx];
			if (fabs(dAngle) < fabs(armors[j].getLightsDeviationAngle())) {
				keep[j]             = false;
				betterLeft[leftIdx] = i;
			} else {
				keep[i] = false;
			}
		} else {
			betterLeft[leftIdx] = i;
		}

		if (betterRight.count(rightIdx)) {
			size_t j = betterRight[rightIdx];
			if (fabs(dAngle) < fabs(armors[j].getLightsDeviationAngle())) {
				keep[j]               = false;
				betterRight[rightIdx] = i;
			} else {
				keep[i] = false;
			}
		} else {
			betterRight[rightIdx] = i;
		}
	}

	vector<ArmorPad> filtered;
	for (size_t i = 0; i < armors.size(); i++) {
		if (keep[i]) { filtered.push_back(std::move(armors[i])); }
	}

	armors = std::move(filtered);

	return true;
}

bool Detector::detect(const Mat& src)
{
	SimpleTimer timer(timeStats);

	src.copyTo(frame);
	// src.copyTo(filteredFrame);

	if (frame.empty()) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Input frame is empty!" << endl;
		}

		state = UNKNOWN_DETECTOR_STATE;

		return false;
	}

	reset();

	if (!preprocess(frame)) { return false; }

	if (!detectLights()) { return false; }

	if (state == LIGHTS_FOUND) { return detectArmors(); }

	return false;
}

}  // namespace armor