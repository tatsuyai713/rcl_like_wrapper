#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp" //標準C++ライブラリ
#include "std_msgs/msg/string.hpp" //ROS2の標準メッセージ

#ifndef STD_MSGS_MSG_STRINGTYPE_HPP
#define STD_MSGS_MSG_STRINGTYPE_HPP
FAST_DDS_DATA_TYPE(std_msgs, msg, String)
#endif // STD_MSGS_MSG_STRINGTYPE_HPP

using namespace std::chrono_literals;

class MyPublisher : public lwrcl::Node
{
public:
  MyPublisher()
  : Node("my_publisher"), count_(0)//ノード名を書く
  {
    lwrcl::dds::TopicQos pub_topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT;
    publisher_ = this->create_publisher<std_msgs::msg::String>(&pub_message_type, "topic", pub_topic_qos); //トピック名を書く。今回はtopicという名前でトピック送信。
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MyPublisher::timer_callback, this)); //500ms間隔でループの指定
  }

private:
  void timer_callback() // Callback
  {
    auto message = std_msgs::msg::String();
    message.data() = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data().c_str());
    publisher_->publish(message);
  }
  lwrcl::TimerBase::SharedPtr timer_;
  lwrcl::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std_msgs::msg::StringType pub_message_type;
};

int main(int argc, char * argv[])
{
  lwrcl::init(argc, argv);
  lwrcl::spin(std::make_shared<MyPublisher>());
  lwrcl::shutdown();
  return 0;
}