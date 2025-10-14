#ifndef __ARMOR_DETECTOR
#define __ARMOR_DETECTOR

#include <opencv2/opencv.hpp>
#include <vector>

#include "armor_detect/armor/armorPad.h"
#include "armor_detect/armor/lightBar.h"
#include "armor_detect/armor/numClassifier.h"

#include "armor_detect/utils/timer.h"

namespace armor {

enum DetectorState
{
	UNDETECTED             = 0x10,
	LIGHTS_FOUND           = 0x11,
	ARMORS_FOUND           = 0x12,
	UNKNOWN_DETECTOR_STATE = 0x13
};

enum DebugInfoFlag : unsigned int
{
	SILENCE     = 0x00,
	/*------Below will in active only when SHOW_FRAME is on------*/
	SHOW_FRAME  = 0x01,
	SHOW_LIGHTS = 0x02,
	SHOW_ARMORS = 0x04,
	SHOW_FPS    = 0x08,
	/*------Above will in active only when SHOW_FRAME is on------*/
	SHOW_BINARY = 0x10,

	SAVE_NUM_IMG = 0x20,  // 保存数字识别的图片

	SHOW_ARMOR_NUM   = 0x040,  // 在新窗口显示
	SHOW_ARMOR_POS   = 0x080,
	SHOW_ARMOR_TYPE  = 0x100,
	SHOW_ARMOR_DIST  = 0x200,
	SHOW_ARMOR_YAW   = 0x400,
	SHOW_ARMOR_PITCH = 0x800,
	ALL_ARMOR_INFO   = 0xFC0,

	TEXT_LIGHTS = 0x1000,
	TEXT_ARMORS = 0x2000,

	ALL_DEBUG_INFO = 0xFFFF
};

bool haveSomeDebugWindowClosed(unsigned flags);

class Detector
{
  private:
	std::vector<LightBar> lights;
	std::vector<ArmorPad> armors;

	cv::Mat frame;
	cv::Mat filteredFrame;
	cv::Mat binary;

	cv::Mat lutTable;

	NumClassifier classifier;

	TimeStats timeStats;

	ArmorColor    enemyColor = UNKNOWN_ARMOR_COLOR;
	DetectorState state      = UNDETECTED;

  private:
	bool preprocess(cv::Mat& src);  // 二值化与形态学处理

	bool detectLights();  // detect light bars
	bool detectArmors();  // match light bars to form armor pads

	bool removeRepeatArmors();

  public:
	void reset();

	bool setEnemyColor(ArmorColor color)
	{
		if (color >= UNKNOWN_ARMOR_COLOR) { return false; }

		enemyColor = color;
		return true;
	}

	bool replaceNumClassifier(NumClassifier&& _classifier)
	{
		if (!_classifier.isModelLoaded()) { return false; }

		classifier = std::move(_classifier);
		return true;
	}

	void setTimeStats(const TimeStats& stats) { timeStats = stats; }

  public:
	std::vector<LightBar> getLights() const
	{
		static const std::vector<LightBar> empty = {};
		return (state > UNDETECTED && state < UNKNOWN_DETECTOR_STATE) ? lights
																	  : empty;
	}

	std::vector<ArmorPad> getArmors() const
	{
		static const std::vector<ArmorPad> empty = {};
		return (state == ARMORS_FOUND) ? armors : empty;
	}

	cv::Mat getFrame() const { return frame.clone(); }
	cv::Mat getBinary() const { return binary.clone(); }

	TimeStats getTimeStats() const { return timeStats; }

	DetectorState getState() const { return state; }
	ArmorColor    getEnemyColor() const { return enemyColor; }

	NumClassifier getNumClassifier() const { return classifier; }

  public:
	bool detect(const cv::Mat& src);  // 主检测函数

	void showDebugInfo(unsigned flags = SILENCE) const;

  public:
	Detector(ArmorColor _color, NumClassifier _classifier = NumClassifier());

	Detector()  = default;
	~Detector() = default;
};

}  // namespace armor

#endif  // __ARMOR_DETECTOR