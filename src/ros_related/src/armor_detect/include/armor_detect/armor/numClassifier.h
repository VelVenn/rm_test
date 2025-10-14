#ifndef __ARMOR_NUM_CLASSIFIER
#define __ARMOR_NUM_CLASSIFIER

#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>

#include "armor_detect/armor/armorPad.h"

namespace armor {
struct NumHOGParam
{
	cv::Size winSize     = cv::Size(32, 32);
	cv::Size blockSize   = cv::Size(8, 8);
	cv::Size blockStride = cv::Size(4, 4);
	cv::Size cellSize    = cv::Size(4, 4);
	int      nbins       = 6;
};

extern NumHOGParam preDefNumHOGParam;

class NumClassifier
{
  private:
	cv::HOGDescriptor    hog;
	cv::Ptr<cv::ml::SVM> svm;

	cv::Mat numImg;
	cv::Mat lutTable;

	NumHOGParam hogParam;

	bool modelLoaded = false;

  private:
	bool initHOG();
	bool initLUT(float gamma);

  public:
	bool setHOGParam(const NumHOGParam& param);
	bool setGamma(float gamma);

  public:
	cv::Mat     getNumImg() const { return numImg.clone(); }
	cv::Mat     getLUT() const { return lutTable.clone(); }
	NumHOGParam getHOGParam() const { return hogParam; }

  public:
	bool loadModel(const std::string& modelPath = "./res/armor_numbers.xml");

	bool createNumRoi(const cv::Mat& frame, ArmorPad& armor);
	int  classify(const cv::Mat& frame, ArmorPad& armor);

  public:
	bool isModelLoaded() const { return modelLoaded; }

  public:
	NumClassifier();
	NumClassifier(NumHOGParam _param, float _gamma);

	~NumClassifier() = default;
};

}  // namespace armor

#endif  // __ARMOR_NUM_CLASSIFIER