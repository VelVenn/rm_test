#include "armor_detect/armor/lightBar.h"

#include <cmath>

#include "armor_detect/armor/armorParam.h"

using namespace cv;

namespace armor {
LightBar::LightBar(const RotatedRect& _box) :
	box(_box),
	center(_box.center),
	length(max(_box.size.width, _box.size.height))
{
	if (box.size.area() > preDefArmorParam.minLightArea) {
		isEmptyLight = false;
	}

	if (_box.angle > 90.0) {
		angle = _box.angle - 180.0;
	} else {
		angle = _box.angle;
	}
}

// assuming the box is aquired from fitEllipse(),
// which means width <= height, θ ∈ [0, 180]
// ref: https://www.programmersought.com/article/5700789323/
bool LightBar::isValid() const
{
	float angle = this->angle;

	if (isEmptyLight) { return false; }

	return box.size.area() > preDefArmorParam.minLightArea &&
		   fabs(angle) < preDefArmorParam.maxLightAngle &&
		   box.size.aspectRatio() > preDefArmorParam.minLightAspectRatio &&
		   box.size.aspectRatio() < preDefArmorParam.maxLightAspectRatio;
}

}  // namespace armor