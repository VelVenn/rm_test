#ifndef __ARMOR_PARAM
#define __ARMOR_PARAM

namespace armor {

struct ArmorParam
{
	int colorThreshold  = 80;   // 图像通道相减二值化所使用的阈值
	int brightThreshold = 165;  // 灰度图二值化阈值

	float minLightArea        = 50.0f;  // 灯条最小面积
	float maxLightAngle       = 45.0f;  // 灯条最大倾斜角
	float minLightSolidity    = 0.5f;   // 灯条最小密实度 
	float maxLightAspectRatio = 1.0f;   // 灯条最大宽高比
	float minLightAspectRatio = 0.1f;   // 灯条最小宽高比

	// float maxLightSolidity    = 1.2f;

	float maxLightsAngleDiff = 6.0f;  // 两灯条倾斜角最大差值

	// 两灯条长度差值与长灯条长度的最大比值
	float maxLightsLengthDiffRatio = 0.5f;
	float maxLightsDeviationAngle = 50.0f;  // 两灯条连线与水平线的最大夹角

	// 两灯条中心点纵坐标差值与灯条平均长度的最大比值
	float maxLightsYDiffRatio = 0.5f;

	// 两灯条中心点横坐标差值与灯条平均长度的最大比值
	float maxLightsXDiffRatio = 4.5f;

	float maxArmorAspectRatio = 4.5f;  // 装甲板最大长宽比
	float minArmorAspectRatio = 1.0f;  // 装甲板最小长宽比

	float minBigArmorAspectRatio = 2.35f;  // 大装甲板最小长宽比
										   // big   -> 235 x 127
										   // small -> 140 x 125

	float maxLightsXDiff = 200.0f;  // 两灯条中心点横坐标最大差值

	ArmorParam()  = default;
	~ArmorParam() = default;
};

struct CameraParam
{
	float loGamma = 0.6f;  // 图像伽马矫正参数
	float hiGamma = 1.3f;

	CameraParam()  = default;
	~CameraParam() = default;
};

extern ArmorParam  preDefArmorParam;
extern CameraParam preDefCameraParam;

}  // namespace armor

#endif  // __ARMOR_PARAM