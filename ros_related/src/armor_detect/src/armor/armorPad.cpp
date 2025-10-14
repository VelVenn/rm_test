#include "armor_detect/armor/armorPad.h"

#include <cmath>

#include "armor_detect/armor/armorParam.h"
#include "armor_detect/armor/lightBar.h"

#include "armor_detect/utils/utils.h"

using namespace cv;
using namespace std;

namespace armor {
static float H_EXPAND_RATIO = 2.6f;  // 灯条外扩比例
static float W_EXPAND_RATIO = 1.2f;  

bool ArmorPad::setVertices()
{
	if (leftLight.isEmpty() || rightLight.isEmpty()) { return false; }

	Size2f exLSize = Size2f(
		leftLight.getBox().size.width * W_EXPAND_RATIO,
		leftLight.getBox().size.height * H_EXPAND_RATIO
	);
	Size2f exRSize = Size2f(
		rightLight.getBox().size.width * W_EXPAND_RATIO,
		rightLight.getBox().size.height * H_EXPAND_RATIO
	);

	RotatedRect exLBox(leftLight.getBox().center, exLSize, this->angle);
	RotatedRect exRBox(rightLight.getBox().center, exRSize, this->angle);

	Point2f exLPts[4], exRPts[4];
	exLBox.points(exLPts);
	exRBox.points(exRPts);

	vertices = { exLPts[3], exLPts[2], exRPts[1], exRPts[0] };

	return true;
}

ArmorPad::ArmorPad(const LightBar& _left, const LightBar& _right) :
	leftLight(_left),
	rightLight(_right)
{
	if (leftLight.isEmpty() || rightLight.isEmpty() ||
		leftLight.getCenter().x > rightLight.getCenter().x) {
		return;
	}

	angle = (leftLight.getAngle() + rightLight.getAngle()) / 2.0f;

	if (!setVertices()) { return; }
	center = getIntersectPoint(vertices);

	length = getDistance(leftLight.getCenter(), rightLight.getCenter());
	width  = (leftLight.getLength() + rightLight.getLength()) / 2.0f;

	type = (length / width > preDefArmorParam.minBigArmorAspectRatio)
			   ? BIG_ARMOR
			   : SMALL_ARMOR;

	isEmptyArmor = false;
}

ArmorPad::ArmorPad(
	const LightBar& _left, const LightBar& _right, int _leftIdx, int _rightIdx
) :
	ArmorPad(_left, _right)
{
	leftIdx  = _leftIdx;
	rightIdx = _rightIdx;
}

/** @return 甲板长宽比，若装甲板为空则返回FLT_MAX */
float ArmorPad::getAspectRatio() const
{
	if (isEmpty()) { return FLT_MAX; }

	return length / width;
}

/** @return 两灯条的角度差，若装甲板为空则返回FLT_MAX */
float ArmorPad::getLightsAngleDiff() const
{
	if (isEmpty()) { return FLT_MAX; }

	return fabs(leftLight.getAngle() - rightLight.getAngle());
}

/** @return 两灯条的长度差比，若装甲板为空则返回FLT_MAX */
float ArmorPad::getLightsLengthDiffRatio() const
{
	if (isEmpty()) { return FLT_MAX; }

	float diff = fabs(leftLight.getLength() - rightLight.getLength());
	float len  = max(leftLight.getLength(), rightLight.getLength());

	return diff / len;
}

/** @return 两灯条连线与水平线的夹角，若装甲板为空则返回FLT_MAX */
float ArmorPad::getLightsDeviationAngle() const
{
	if (isEmpty()) { return FLT_MAX; }

	// [-90, 90]
	float deltaY = rightLight.getCenter().y - leftLight.getCenter().y;
	float deltaX = rightLight.getCenter().x - leftLight.getCenter().x;

	// ref: https://en.wikipedia.org/wiki/Atan2
	return fabs(atan2(deltaY, deltaX) * 180.0f / CV_PI);
}

/** @return 两灯条中心点横坐标差值与灯条平均长度的比值，
 *  		若装甲板为空则返回FLT_MAX
 */
float ArmorPad::getLightsXDiffRatio() const
{
	if (isEmpty()) { return FLT_MAX; }

	float deltaX = fabs(rightLight.getCenter().x - leftLight.getCenter().x);
	float len    = (leftLight.getLength() + rightLight.getLength()) / 2.0f;

	return deltaX / len;
}

/** @return 两灯条中心点纵坐标差值与灯条平均长度的比值，
 *  		若装甲板为空则返回FLT_MAX
 */
float ArmorPad::getLightsYDiffRatio() const
{
	if (isEmpty()) { return FLT_MAX; }

	float deltaY = fabs(rightLight.getCenter().y - leftLight.getCenter().y);
	float len    = (leftLight.getLength() + rightLight.getLength()) / 2.0f;

	return deltaY / len;
}

bool ArmorPad::isSuitable() const
{
	if (isEmpty()) { return false; }

	return getAspectRatio() > preDefArmorParam.minArmorAspectRatio &&
		   getAspectRatio() < preDefArmorParam.maxArmorAspectRatio &&
		   getLightsAngleDiff() < preDefArmorParam.maxLightsAngleDiff &&
		   getLightsLengthDiffRatio() <
			   preDefArmorParam.maxLightsLengthDiffRatio &&
		   getLightsDeviationAngle() <
			   preDefArmorParam.maxLightsDeviationAngle &&
		   getLightsXDiffRatio() < preDefArmorParam.maxLightsXDiffRatio &&
		   getLightsYDiffRatio() < preDefArmorParam.maxLightsYDiffRatio;
}

};  // namespace armor