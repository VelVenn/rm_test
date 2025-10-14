#include "armor_detect/utils/utils.h"

#include <cmath>

using namespace cv;
using namespace std;

namespace armor {

float getDistance(const Point2f& p1, const Point2f& p2)
{
	return hypot(p1.x - p2.x, p1.y - p2.y);
}

/**
 * @brief Calculate the intersection point of two lines
 * @param points An array of four points in the order of bottom-left, top-left,
 * top-right, bottom-right
 *
 * @return The intersection point of the two lines, or Point2f(FLT_MAX, FLT_MAX)
 * if the lines are parallel
 */
Point2f getIntersectPoint(const array<Point2f, 4>& points)
{
	// Line 1: p1 -> p2 (diagonal from points[0] to points[2])
	Point2f p1 = points[0];
	Point2f p2 = points[2];

	// Line 2: p3 -> p4 (diagonal from points[1] to points[3])
	Point2f p3 = points[1];
	Point2f p4 = points[3];

	float d = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);

	// Parallel when d is close to 0
	if (abs(d) < 1e-6) { return Point2f(FLT_MAX, FLT_MAX); }

	float t =
		((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / d;

	float ix = p1.x + t * (p2.x - p1.x);
	float iy = p1.y + t * (p2.y - p1.y);

	return Point2f(ix, iy);
}

DebugLevel debugLevel = ERROR;

}  // namespace armor