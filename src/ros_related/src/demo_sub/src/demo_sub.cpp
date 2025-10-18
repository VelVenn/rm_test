#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <functional>
#include <memory>

#include <opencv2/opencv.hpp>

using namespace rclcpp;
using namespace std;
using namespace std::placeholders;

class MinimalCVSubscriber : public Node {
private:
    Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
    Subscription<std_msgs::msg::String>::SharedPtr str_subscription_;

    cv::Mat raw_image_;
    string raw_str_;

private:
    void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        try {
            raw_image_ = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
            cv::imshow("view", raw_image_);

            if (cv::waitKey(30) == 27) {
                RCLCPP_INFO(this->get_logger(), "Esc Key pressed, shutting down node.");
                rclcpp::shutdown();
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    void str_callback(const std_msgs::msg::String::SharedPtr str_msg)
    {
        raw_str_ = str_msg->data;
        RCLCPP_INFO(this->get_logger(), "Received string: '%s'", raw_str_.c_str());
    }

public:
    MinimalCVSubscriber()
        : Node("cv_subscriber")
    {
        img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "rand_px", 10, std::bind(&MinimalCVSubscriber::img_callback, this, _1));
            
        str_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "sim_str", 10, std::bind(&MinimalCVSubscriber::str_callback, this, _1));
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalCVSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
