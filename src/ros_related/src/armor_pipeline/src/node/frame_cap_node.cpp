#include "rclcpp/rclcpp.hpp"

#include "armor_pipeline/frame_capture.h"

using namespace rclcpp;
using namespace std;

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<armor_pipeline::FrameCapture>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}