#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MySubscriber : public lwrcl::Node  //MySubscriberクラス
{
  public:
    MySubscriber()
    : Node("my_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MySubscriber::topic_callback, this, _1));//topicという名前のトピックを購読、バッファ10
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    lwrcl::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  lwrcl::init(argc, argv);
  lwrcl::spin(std::make_shared<MySubscriber>());
  lwrcl::shutdown();
  return 0;
}