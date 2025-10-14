#ifndef __ARMOR_UTILS
#define __ARMOR_UTILS

#include <array>
#include <opencv2/opencv.hpp>

namespace armor {

enum DebugLevel
{
	NONE    = 0x90,
	ERROR   = 0x91,
	INFO    = 0x92,
	VERBOSE = 0x93
} extern debugLevel;

/** @brief Get the distance between two points */
float getDistance(const cv::Point2f& p1, const cv::Point2f& p2);

cv::Point2f getIntersectPoint(const std::array<cv::Point2f, 4>& points);

}  // namespace armor

#endif  // __ARMOR_UTILS