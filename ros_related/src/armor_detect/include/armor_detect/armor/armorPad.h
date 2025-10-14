#ifndef __ARMOR_PAD
#define __ARMOR_PAD

#include <array>
#include <opencv2/opencv.hpp>

#include "armor_detect/armor/lightBar.h"

namespace armor {

enum ArmorColor
{
	BLUE_ARMOR          = 0x00,
	RED_ARMOR           = 0x01,
	UNKNOWN_ARMOR_COLOR = 0x02
};

enum ArmorType
{
	SMALL_ARMOR        = 0x20,
	BIG_ARMOR          = 0x21,
	UNKNOWN_ARMOR_TYPE = 0x22
};

class ArmorPad
{
  private:
	LightBar leftLight, rightLight;
	int      leftIdx = -1, rightIdx = -1;  // 左右灯条在Detector类灯条数组中的索引,
		// 单独使用ArmorPad类时无意义

	// Rect roi;

	cv::Mat image;  // 裁剪出的装甲板图像
	int     number = -1;

	std::array<cv::Point2f, 4> vertices;
	cv::Point2f                center;

	float angle;
	float length;
	float width;

	ArmorType type = UNKNOWN_ARMOR_TYPE;

	bool isEmptyArmor = true;

  private:
	bool setVertices();

  public:
	void setLeftIdx(int idx) { leftIdx = idx; }
	void setRightIdx(int idx) { rightIdx = idx; }

	void setNumber(int num) { number = num; }
	void setImage(const cv::Mat& img) { img.copyTo(image); }

  public:
	int getLeftIdx() const { return leftIdx; }
	int getRightIdx() const { return rightIdx; }

	cv::Mat getImage() const { return image.clone(); }
	int     getNumber() const { return number; }

	std::array<cv::Point2f, 4> getVertices() const
	{
		static const std::array<cv::Point2f, 4> empty = {};
		return isEmptyArmor ? empty : vertices;
	}

	cv::Point2f getCenter() const { return center; }

	float getAngle() const { return angle; }
	float getLength() const { return length; }
	float getWidth() const { return width; }

	ArmorType getType() const { return type; }

  public:
	// 获得装甲板长宽比
	float getAspectRatio() const;

	// 获得两灯条倾斜角差值
	float getLightsAngleDiff() const;

	// 获得两灯条长度差值与长灯条长度的比值
	float getLightsLengthDiffRatio() const;

	// 获得两灯条连线与水平线的夹角
	float getLightsDeviationAngle() const;

	// 获得两灯条中心点横坐标差值与灯条平均长度的比值
	float getLightsXDiffRatio() const;

	// 获得两灯条中心点纵坐标差值与灯条平均长度的比值
	float getLightsYDiffRatio() const;

  public:
	bool isEmpty() const { return isEmptyArmor; }

	bool isSuitable() const;

  public:
	ArmorPad(
		const LightBar& _left, const LightBar& _right, int _leftIdx,
		int _rightIdx
	);

	ArmorPad(const LightBar& _left, const LightBar& _right);

	~ArmorPad() = default;
};

}  // namespace armor

#endif  // __ARMOR_PAD
