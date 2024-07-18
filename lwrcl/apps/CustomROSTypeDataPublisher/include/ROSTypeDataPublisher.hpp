#ifndef ROSTYPEDATAPUBLISHER_H_
#define ROSTYPEDATAPUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "CustomROSTypeDataPublisher/msg/custom_message.hpp"
#include <functional>
#include <memory>
#include <string>

using namespace rclcpp;

class ROSTypeDataPublisher : public Node
{
public:
  ROSTypeDataPublisher(uint16_t domain_number);
  ROSTypeDataPublisher(std::string node_name);
  ROSTypeDataPublisher(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
  virtual ~ROSTypeDataPublisher();

  // Override init and run methods from Node
  // void run() override;

  bool init_config(const std::string &config_file_path);

  // Callback function to publish data
  void callbackPublish(int test);

private:
  std::string topic_name_;
  uint16_t interval_ms_;
  Publisher<CustomROSTypeDataPublisher::msg::CustomMessage>::SharedPtr publisher_ptr_;
  TimerBase::SharedPtr timer_ptr_;
  std::function<void()> timer_callback_;
};

#endif /* ROSTYPEDATAPUBLISHER_H_ */
