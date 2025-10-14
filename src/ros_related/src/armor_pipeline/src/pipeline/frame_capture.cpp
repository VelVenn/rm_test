#include "armor_pipeline/frame_capture.h"

#include <filesystem>

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;

namespace armor_pipeline {
FrameCapture::FrameCapture(const string& name) : Node(name)
{
	RCLCPP_INFO(this->get_logger(), "%s has been created", name.c_str());

	this->declare_parameter("vid_path", "");
	this->declare_parameter("playing", false);
	this->declare_parameter("loop_display", false);

	frame_pub_ =
		this->create_publisher<sensor_msgs::msg::Image>("vid_frame", 10);

	playing_stat_pub_ =
		this->create_publisher<std_msgs::msg::Bool>("playing_stat", 10);

	timer_ = this->create_wall_timer(
		33ms, std::bind(&FrameCapture::timer_callback, this)
	);
}

FrameCapture::~FrameCapture()
{
	cap_.release();
	RCLCPP_INFO(this->get_logger(), "%s has been destroyed", this->get_name());
}

void FrameCapture::timer_callback()
{
	string vid_path     = this->get_parameter("vid_path").as_string();
	bool   playing      = this->get_parameter("playing").as_bool();
	bool   loop_display = this->get_parameter("loop_display").as_bool();

	playing_stat_pub_->publish(std_msgs::msg::Bool().set__data(playing));

	if (is_single_image_) {
		auto msg =
			cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", raw_image_)
				.toImageMsg();

		frame_pub_->publish(*msg);
		this->set_parameter(Parameter("playing", false));
	}

	if (vid_path != cur_vid_path_ && !vid_path.empty()) {
		if (!filesystem::exists(filesystem::path(vid_path))) {
			RCLCPP_ERROR(
				this->get_logger(), "File does not exist: %s", vid_path.c_str()
			);

			this->set_parameter(Parameter("vid_path", cur_vid_path_));
		} else {
			cv::VideoCapture new_cap(vid_path);

			if (!new_cap.isOpened()) {
				RCLCPP_ERROR(
					this->get_logger(), "Failed to open video file: %s",
					vid_path.c_str()
				);

				this->set_parameter(Parameter("vid_path", cur_vid_path_));
			} else {
				RCLCPP_INFO(
					this->get_logger(), "Opened new video: %s", vid_path.c_str()
				);

				cap_          = std::move(new_cap);
				cur_vid_path_ = vid_path;

				if (cap_.get(cv::CAP_PROP_FRAME_COUNT) <= 1) {
					is_single_image_ = true;
					cap_ >> raw_image_;

					return;
				} else {
					is_single_image_ = false;
				}

				if (!playing) {
					this->set_parameter(Parameter("playing", true));
					playing = true;
				}
			}
		}
	}

	if (playing) {
		if (!cap_.isOpened()) {
			RCLCPP_WARN(this->get_logger(), "No video file is opened");
			this->set_parameter(Parameter("playing", false));
			return;
		}

		cap_ >> raw_image_;

		if (raw_image_.empty()) {
			if (loop_display) {
				cap_.set(cv::CAP_PROP_POS_FRAMES, 0);  // 单张图片时不存在帧序列
				cap_ >> raw_image_;
			} else {
				RCLCPP_INFO(
					this->get_logger(), "End of video file reached: %s",
					vid_path.c_str()
				);

				this->set_parameter(Parameter("playing", false));
				return;
			}
		}

		auto msg =
			cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", raw_image_)
				.toImageMsg();

		frame_pub_->publish(*msg);
	}
}

}  // namespace armor_pipeline