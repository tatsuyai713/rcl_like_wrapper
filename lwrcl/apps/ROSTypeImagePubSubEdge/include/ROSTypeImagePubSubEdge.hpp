#ifndef ROSTYPEIMAGEPUBLSUBEDGE_H_
#define ROSTYPEIMAGEPUBLSUBEDGE_H_

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

class ROSTypeImagePubSubEdge : public Node
{
public:
  ROSTypeImagePubSubEdge(uint16_t domain_number);
  ROSTypeImagePubSubEdge(std::string node_name);
  ROSTypeImagePubSubEdge(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
  virtual ~ROSTypeImagePubSubEdge();

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
  sensor_msgs::msg::ImageType pub_message_type_;
  sensor_msgs::msg::ImageType sub_message_type_;
  int counter_;
  sensor_msgs::msg::Image::SharedPtr edge_msg_;
};

#endif /* ROSTYPEIMAGEPUBLSUBEDGE_H_ */
