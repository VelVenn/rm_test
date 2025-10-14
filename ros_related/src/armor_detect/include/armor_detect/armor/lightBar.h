#ifndef __ARMOR_LIGHT_BAR
#define __ARMOR_LIGHT_BAR

#include <cmath>
#include <opencv2/opencv.hpp>

namespace armor {

class LightBar
{
  private:
	cv::RotatedRect box;
	cv::Point2f center;
	float angle;  // 左偏 [-90, 0], 右偏 [0, 90]
	float length;

	bool isEmptyLight = true;

  public:
	cv::RotatedRect getBox() const { return box; }
	cv::Point2f getCenter() const { return center; }
	float getAngle() const { return angle; }
	float getLength() const { return length; }

  public:
	bool isEmpty() const { return isEmptyLight; }

	bool isValid() const;

  public:
	LightBar(const cv::RotatedRect& _box);

	LightBar() = default;
	~LightBar() = default;
};

}  // namespace armor

#endif  // __ARMOR_LIGHT_BAR