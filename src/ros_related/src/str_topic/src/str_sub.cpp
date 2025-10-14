#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <functional>
#include <memory>
#include <string>

using namespace rclcpp;
using namespace std;
using namespace std::placeholders;

class StrSubscriber : public Node
{
  public:
    StrSubscriber(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Subscriber created: %s", this->get_name());

        str_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "sim_str", 10, bind(&StrSubscriber::topic_callback, this, _1));
    }

    string get_raw_msg() const
    {
        return raw_msg_;
    }

  private:
    void topic_callback(const shared_ptr<std_msgs::msg::String> msg)
    {
        raw_msg_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    shared_ptr<Subscription<std_msgs::msg::String>> str_subscription_;

    string raw_msg_;
};

int main(int argc, char *argv[])
{
    init(argc, argv);
    auto node = make_shared<StrSubscriber>("str_subscriber");
    spin(node);
    shutdown();

    return 0;
}