#include "armor_pipeline/armor_finder.h"

#include "armor_detect/utils/timer.h"

#include <filesystem>

using namespace rclcpp;
using namespace rcl_interfaces::msg;

using namespace std;
using namespace std::placeholders;

using namespace armor;

namespace armor_pipeline {

ArmorFinder::ArmorFinder(const string& name) : Node(name), detector_()
{
	RCLCPP_INFO(this->get_logger(), "%s has been created", name.c_str());

	this->declare_parameter("enemy_color", "");
	this->declare_parameter("model_path", "");

	frame_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		"vid_frame", 10, std::bind(&ArmorFinder::frame_callback, this, _1)
	);

	playing_stat_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"playing_stat", 10,

		[this](const std_msgs::msg::Bool::SharedPtr msg) {
			bool prev_status = is_playing_;
			is_playing_      = msg->data;

			// 使用GTK, 当视频暂停播放并恢复时，Wayland可能会直接断开连接，导致程序崩溃
			// X11下不会出现这个问题
			// Using WSLg

			// if (!prev_status && is_playing_) {
			// 	cv::destroyAllWindows();

			// 	cv::Mat blank(540, 480, CV_8UC3, cv::Scalar(0, 0, 0));
			// 	cv::namedWindow("Frame", cv::WINDOW_NORMAL);
			// 	cv::namedWindow("Binary", cv::WINDOW_NORMAL);
			// 	cv::namedWindow("Armor Info", cv::WINDOW_NORMAL);

			// 	cv::imshow("Frame", blank);
			// 	cv::imshow("Binary", blank);
			// 	cv::imshow("Armor Info", blank);
			// 	cv::waitKey(1);

			// 	RCLCPP_INFO(
			// 		this->get_logger(),
			// 		"Video replay detected, rebuilding windows"
			// 	);
			// }
		}
	);

	param_handler_ = this->add_on_set_parameters_callback(
		std::bind(&ArmorFinder::param_callback, this, _1)
	);
}

ArmorFinder::~ArmorFinder()
{
	RCLCPP_INFO(this->get_logger(), "%s has been destroyed", this->get_name());
	cv::destroyAllWindows();
}

SetParametersResult ArmorFinder::param_callback(
	const std::vector<rclcpp::Parameter>& params
)
{
	SetParametersResult result;
	result.successful = true;
	result.reason     = "Params set successfully.";

	for (const auto& param : params) {
		if (param.get_name() == "enemy_color") {
			string new_color = param.as_string();

			if (new_color == "red" || new_color == "blue") {
				enemy_color_ = (new_color == "red") ? RED_ARMOR : BLUE_ARMOR;
				detector_.setEnemyColor(enemy_color_);

				RCLCPP_INFO(
					this->get_logger(), "Set enemy color to: %s",
					new_color.c_str()
				);
			} else {
				result.successful = false;
				result.reason     = "Invalid enemy color: " +
								(new_color.empty() ? "<empty>" : new_color);

				RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
				break;
			}

			continue;
		} else if (param.get_name() == "model_path") {
			string new_model_path = param.as_string();

			if (new_model_path.empty()) {
				result.successful = false;
				result.reason     = "Model path cannot be empty.";

				RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
				break;
			}

			if (!filesystem::exists(filesystem::path(new_model_path))) {
				result.successful = false;
				result.reason     = "File does not exist: " + new_model_path;

				RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
				break;
			}

			NumClassifier new_classifier = NumClassifier();
			if (new_classifier.loadModel(new_model_path)) {
				detector_.replaceNumClassifier(std::move(new_classifier));
				model_path_ = new_model_path;

				RCLCPP_INFO(
					this->get_logger(), "Loaded new model: %s",
					new_model_path.c_str()
				);
			} else {
				result.successful = false;
				result.reason     = "Failed to load model: " + new_model_path;

				RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
				break;
			}

			continue;
		} else {
			result.successful = false;
			result.reason     = "Unknown parameter: " + param.get_name();

			RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
			break;
		}
	}

	return result;
}

void ArmorFinder::frame_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
		raw_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	auto cur_time = detector_.getTimeStats();

	if (!raw_image_.empty() &&
		detector_.getEnemyColor() != UNKNOWN_ARMOR_COLOR) {
		detector_.detect(raw_image_);

		unsigned flags = 0xFF - SAVE_NUM_IMG;

		if (!is_playing_) { detector_.setTimeStats(cur_time); }

		detector_.showDebugInfo(flags);

		if (cv::waitKey(10) == 27) {
			RCLCPP_INFO(
				this->get_logger(), "Esc Key pressed, shutting down node."
			);
			rclcpp::shutdown();

			return;
		}
	}
}

}  // namespace armor_pipeline
