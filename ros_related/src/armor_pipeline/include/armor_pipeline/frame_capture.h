#ifndef __VID_FRAME_CAPTURE
#define __VID_FRAME_CAPTURE

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#include <opencv2/opencv.hpp>

namespace armor_pipeline {

class FrameCapture : public rclcpp::Node
{
  private:
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     playing_stat_pub_;
	rclcpp::TimerBase::SharedPtr                          timer_;

	cv::Mat          raw_image_;
	cv::VideoCapture cap_;

	std::string cur_vid_path_ = "";

	bool is_single_image_ = false;

  private:
	void timer_callback();

  public:
	FrameCapture(const std::string& name = "frame_capture");
	~FrameCapture() override;
};

}  // namespace armor_pipeline

#endif  // __VID_FRAME_CAPTURE