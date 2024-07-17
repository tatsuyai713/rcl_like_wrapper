#ifndef ROSTYPEIMAGEPUBLSUB_H_
#define ROSTYPEIMAGEPUBLSUB_H_

#include "lwrcl.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <functional>
#include <memory>
#include <string>

using namespace lwrcl;

#ifndef SENSOR_MSGS_MSG_IMAGETYPE_HPP
#define SENSOR_MSGS_MSG_IMAGETYPE_HPP
FAST_DDS_DATA_TYPE(sensor_msgs, msg, Image)
#endif // SENSOR_MSGS_MSG_IMAGETYPE_HPP

class ROSTypeImagePubSub : public Node
{
public:
  ROSTypeImagePubSub(uint16_t domain_number);
  ROSTypeImagePubSub(std::string node_name);
  ROSTypeImagePubSub(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
  virtual ~ROSTypeImagePubSub();

  // Override init and run methods from Node
  // void run() override;

  bool init_config(const std::string &config_file_path);

  // Callback function to publish data
  void callbackPublish(int test);

  // Callback function to subscribe data
  void callbackSubscribe(sensor_msgs::msg::Image::SharedPtr message);

private:
  std::string publish_topic_name_;
  std::string subscribe_topic_name_;
  uint16_t interval_ms_;
  Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_ptr_;
  Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_ptr_;
  TimerBase::SharedPtr timer_ptr_;
  std::function<void()> timer_callback_;
  sensor_msgs::msg::ImageType::SharedPtr pub_message_type_;
  sensor_msgs::msg::ImageType::SharedPtr sub_message_type_;
  int counter_;
};

#endif /* ROSTYPEIMAGEPUBLSUB_H_ */
