#include "rclcpp/rclcpp.hpp"

#include "armor_pipeline/armor_finder.h"

#include "armor_detect/utils/utils.h"

using namespace rclcpp;
using namespace std;

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	armor::debugLevel = armor::DebugLevel::NONE;

	auto node = std::make_shared<armor_pipeline::ArmorFinder>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}