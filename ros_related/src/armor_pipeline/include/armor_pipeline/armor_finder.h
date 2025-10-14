#ifndef __VID_FRAME_CHECK
#define __VID_FRAME_CHECK

#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#include <opencv2/opencv.hpp>

#include "armor_detect/armor/armorPad.h"
#include "armor_detect/armor/armorParam.h"
#include "armor_detect/armor/detector.h"
#include "armor_detect/armor/numClassifier.h"

#include "armor_detect/utils/timer.h"
#include "armor_detect/utils/utils.h"

namespace armor_pipeline {

class ArmorFinder : public rclcpp::Node
{
  private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     playing_stat_sub_;

	cv::Mat raw_image_;

	armor::Detector detector_;

	armor::ArmorColor enemy_color_ = armor::UNKNOWN_ARMOR_COLOR;
	std::string       model_path_  = "";

	cv::Size display_size_;

	bool is_playing_ = false;

	OnSetParametersCallbackHandle::SharedPtr param_handler_;

  private:
	void frame_callback(const sensor_msgs::msg::Image::SharedPtr msg);

	void playing_stat_callback(const std_msgs::msg::Bool::SharedPtr msg);

	rcl_interfaces::msg::SetParametersResult param_callback(
		const std::vector<rclcpp::Parameter>& params
	);

  public:
	ArmorFinder(const std::string& name = "armor_finder");
	~ArmorFinder() override;
};

}  // namespace armor_pipeline

#endif  // __VID_FRAME_CHECK