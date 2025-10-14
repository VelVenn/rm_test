#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include <opencv2/opencv.hpp>

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;

class MininalCVPublisher : public Node
{
  private:
	Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
	TimerBase::SharedPtr                          timer_;

	cv::Mat raw_image_ = cv::Mat::zeros(480, 640, CV_8UC3);

  private:
	void timer_callback()
	{
		cv::randu(raw_image_, cv::Scalar::all(0), cv::Scalar::all(255));
		auto msg =
			cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", raw_image_)
				.toImageMsg();

		// only accept shared_ptr<const T> as referenced argument
		img_publisher_->publish(*msg);
	}

  public:
	MininalCVPublisher() : Node("cv_publisher")
	{
		img_publisher_ =
			this->create_publisher<sensor_msgs::msg::Image>("rand_px", 10);

		timer_ = this->create_wall_timer(
			500ms, std::bind(&MininalCVPublisher::timer_callback, this)
		);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MininalCVPublisher>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}