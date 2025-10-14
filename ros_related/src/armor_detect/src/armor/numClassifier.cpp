#include "armor_detect/armor/numClassifier.h"

#include <array>

#include "armor_detect/armor/armorParam.h"

#include "armor_detect/utils/utils.h"

using namespace cv;
using namespace cv::ml;
using namespace std;

namespace armor {

static float W_MODIFY_RATE   = 1.0f;  // 宽度修正系数
static float L_MODIFY_RATE_S = 1.0f;  // 小甲板长度修正系数
static float L_MODIFY_RATE_B = 0.5f;  // 大甲板长度修正系数

NumHOGParam preDefNumHOGParam = NumHOGParam();

NumClassifier::NumClassifier(NumHOGParam _param, float _gamma) :
	hogParam(_param)
{
	if (!initHOG() || !initLUT(_gamma)) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Failed to initialize NumClassifier!" << endl;
		}

		return;
	}
}

NumClassifier::NumClassifier() :
	NumClassifier(preDefNumHOGParam, preDefCameraParam.loGamma)
{}

bool NumClassifier::initHOG()
{
	hog = HOGDescriptor(
		hogParam.winSize, hogParam.blockSize, hogParam.blockStride,
		hogParam.cellSize, hogParam.nbins
	);

	if (hog.getDescriptorSize() <= 0) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Failed to initialize HOG descriptor!" << endl;
		}
		return false;
	}

	return true;
}

bool NumClassifier::initLUT(float gamma)
{
	if (gamma <= 0.0f) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Gamma must be positive!" << endl;
		}

		return false;
	}

	lutTable       = Mat(1, 256, CV_8UC1);
	uchar* lutData = lutTable.data;

	for (int i = 0; i < 256; i++) {
		lutData[i] = saturate_cast<uchar>(pow(i / 255.0f, gamma) * 255.0f);
	}

	return true;
}

bool NumClassifier::loadModel(const string& modelPath)
{
	svm = Algorithm::load<SVM>(modelPath);

	if (svm.empty()) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Failed to load SVM model from: " << modelPath
				 << "!" << endl;
		}

		modelLoaded = false;
		return false;
	}

	modelLoaded = true;
	return true;
}

bool NumClassifier::createNumRoi(const Mat& frame, ArmorPad& armor)
{
	if (frame.empty() || armor.isEmpty()) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Input frame or armor is empty!" << endl;
		}

		return false;
	}

	// Point2f center = armor.getCenter();

	int wid = static_cast<int>(armor.getWidth() * W_MODIFY_RATE);

	float lRate =
		armor.getType() == BIG_ARMOR ? L_MODIFY_RATE_B : L_MODIFY_RATE_S;
	int len = static_cast<int>(armor.getLength() * lRate);

	// Size        idealSize = Size(len, wid);
	// RotatedRect idealRect =
	// 	RotatedRect(armor.getCenter(), idealSize, armor.getAngle());

	// Point2f idealPts[4];
	// idealRect.points(idealPts);

	auto vertices = armor.getVertices();

	array<Point2f, 4> worldPts = { vertices[1], vertices[2], vertices[3],
								   vertices[0] };

	array<Point2f, 4> dstPts = { Point2f(0.0f, 0.0f), Point2f(len - 1.0f, 0.0f),
								 Point2f(len - 1.0f, wid - 1.0f),
								 Point2f(0.0f, wid - 1.0f) };

	Rect edge = boundingRect(worldPts) & Rect(0, 0, frame.cols, frame.rows);

	if (edge.empty()) {
		if (debugLevel >= INFO) {
			cerr << "[INFO] Cannot find valid ROI for number classification!"
				 << endl;
		}

		return false;
	}

	array<Point2f, 4> localPts;
	for (int i = 0; i < 4; i++) {
		localPts[i] =
			worldPts[i] -
			Point2f(static_cast<float>(edge.x), static_cast<float>(edge.y));
	}

	Mat transMat = getPerspectiveTransform(localPts, dstPts);

	Mat numRoi = frame(edge);

	numImg = Mat::zeros(Size(len, wid), CV_8UC1);
	Mat numGray;
	cvtColor(numRoi, numGray, COLOR_BGR2GRAY);

	warpPerspective(numGray, numImg, transMat, Size(len, wid));

	if (numImg.empty()) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Affine transform failed!" << endl;
		}

		return false;
	}

	LUT(numImg, lutTable, numImg);

	resize(numImg, numImg, hogParam.winSize);

	armor.setImage(numImg.clone());

	return true;
}

int NumClassifier::classify(const Mat& frame, ArmorPad& armor)
{
	if (!modelLoaded) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] SVM model not loaded!" << endl;
		}

		return -1;
	}

	if (frame.empty() || armor.isEmpty()) {
		if (debugLevel >= ERROR) {
			cerr << "[ERROR] Input frame or armor is empty!" << endl;
		}

		return -1;
	}

	if (!createNumRoi(frame, armor)) { return -1; }

	vector<float> descriptors;
	hog.compute(numImg, descriptors);

	Mat descriptorMat = Mat(descriptors).reshape(1, 1);

	int result = static_cast<int>(svm->predict(descriptorMat));

	if (result <= 0) { return -1; }

	return result;
}

bool NumClassifier::setHOGParam(const NumHOGParam& param)
{
	hogParam = param;
	return initHOG();
}

bool NumClassifier::setGamma(float gamma) 
{
	return initLUT(gamma);
}

}  // namespace armor