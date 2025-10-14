#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional>
#include <memory>

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;

class StrPublisher : public Node
{
  private:
    Publisher<std_msgs::msg::String>::SharedPtr str_publisher_;
    TimerBase::SharedPtr timer_;

    std_msgs::msg::String message_ = std_msgs::msg::String();

    int counter_ = 0;
    int interval_; // milliseconds

  private:
    void publish_message()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s', %d", message_.data.c_str(), ++this->counter_);

        str_publisher_->publish(message_);
    }

  public:
    StrPublisher(const string &name, const string &msg = "Empty", int interval = 1000) : Node(name), interval_(interval)
    {
        RCLCPP_INFO(this->get_logger(), "Node created: %s", this->get_name());

        str_publisher_ = this->create_publisher<std_msgs::msg::String>("sim_str", 10);

        message_.data = msg;

        timer_ = this->create_wall_timer(interval_ * 1ms, bind(&StrPublisher::publish_message, this));
    }

    void set_message(const string &msg)
    {
        message_.data = msg;
    }
};

int main(int argc, char *argv[])
{
    init(argc, argv);

    string node_name = "str_publisher";
    string msg = "hello";
    int interval = 500; // milliseconds

    auto node = make_shared<StrPublisher>(node_name, msg, interval);
    spin(node);
    shutdown();

    return 0;
}