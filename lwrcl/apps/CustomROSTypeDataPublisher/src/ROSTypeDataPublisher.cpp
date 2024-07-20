#include "ROSTypeDataPublisher.hpp"
#include <iostream>
#include <chrono>
#include <yaml-cpp/yaml.h>

ROSTypeDataPublisher::ROSTypeDataPublisher(uint16_t domain_number)
    : Node(domain_number), topic_name_("default_topic"), interval_ms_(1000)
{
}

ROSTypeDataPublisher::ROSTypeDataPublisher(const std::string &node_name)
    : Node(node_name), topic_name_("default_topic"), interval_ms_(1000)
{
}

ROSTypeDataPublisher::ROSTypeDataPublisher(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant)
    : Node(participant), topic_name_("default_topic"), interval_ms_(1000)
{
}

ROSTypeDataPublisher::ROSTypeDataPublisher(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant, const std::string &node_name)
    : Node(participant, node_name), topic_name_("default_topic"), interval_ms_(1000)
{
}

ROSTypeDataPublisher::~ROSTypeDataPublisher()
{
}

bool ROSTypeDataPublisher::init_config(const std::string &config_file_path)
{
  // Load configuration from YAML file
  YAML::Node node = YAML::LoadFile(config_file_path);
  YAML::Node config = node["config"];

  for (const auto &topic_node : config["topics"])
  {
    topic_name_ = topic_node["name"].as<std::string>("default_topic");
    interval_ms_ = topic_node["interval_ms"].as<uint16_t>(1000);
    std::cout << "Topic name: " << topic_name_ << ", Interval: " << interval_ms_ << " ms" << std::endl;
  }

  if (interval_ms_ <= 0)
  {
    std::cerr << "Interval Time Error!" << std::endl;
    return false;
  }

  publisher_ptr_ = create_publisher<CustomROSTypeDataPublisher::msg::CustomMessage>(topic_name_, 10);
  if (!publisher_ptr_)
  {
    std::cerr << "Error: Failed to create a publisher." << std::endl;
    return false;
  }

  int test = 100;

  // Setup timer for periodic callback
  timer_callback_ = [this, test]()
  { this->callbackPublish(test); };
  timer_ptr_ = create_wall_timer(std::chrono::milliseconds(interval_ms_), timer_callback_);
  if (!timer_ptr_)
  {
    std::cerr << "Error: Failed to create a timer." << std::endl;
    return false;
  }

  return true;
}

void ROSTypeDataPublisher::callbackPublish(int test)
{
  // Update and publish message
  std::shared_ptr<CustomROSTypeDataPublisher::msg::CustomMessage> publish_msg = std::make_shared<CustomROSTypeDataPublisher::msg::CustomMessage>();
  publish_msg->index(publish_msg->index() + 1);
  std::string s = "BigData" + std::to_string(publish_msg->index() % 10);
  publish_msg->message(s);

  publisher_ptr_->publish(publish_msg);
}
