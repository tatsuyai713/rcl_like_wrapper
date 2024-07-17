#ifndef ROSTYPEIMAGEPUBLSUBMONO_H_
#define ROSTYPEIMAGEPUBLSUBMONO_H_

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
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

class ROSTypeImagePubSubMono : public Node
{
public:
  ROSTypeImagePubSubMono(uint16_t domain_number);
  ROSTypeImagePubSubMono(std::string node_name);
  ROSTypeImagePubSubMono(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
  virtual ~ROSTypeImagePubSubMono();

  bool init_config(const std::string &config_file_path);
  // Callback function to subscribe data
  void callbackSubscribe(sensor_msgs::msg::Image::SharedPtr message);

private:
  std::string publish_topic_name_;
  std::string subscribe_topic_name_;
  uint16_t interval_ms_;
  Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_ptr_;
  Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_ptr_;
  TimerBase::SharedPtr timer_ptr_;
  sensor_msgs::msg::ImageType::SharedPtr pub_message_type_;
  sensor_msgs::msg::ImageType::SharedPtr sub_message_type_;
  int counter_;
  sensor_msgs::msg::Image::SharedPtr gray_msg_;
};

#endif /* ROSTYPEIMAGEPUBLSUBMONO_H_ */
