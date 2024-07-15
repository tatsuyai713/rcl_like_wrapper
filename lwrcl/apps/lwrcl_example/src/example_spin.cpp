#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace rclcpp;

#ifndef SENSOR_MSGS_MSG_IMAGETYPE_HPP
#define SENSOR_MSGS_MSG_IMAGETYPE_HPP
FAST_DDS_DATA_TYPE(sensor_msgs, msg, Image)
#endif // SENSOR_MSGS_MSG_IMAGETYPE_HPP

// Callback function for handling received messages
void myCallbackFunction(sensor_msgs::msg::Image::SharedPtr message)
{
  if (message == nullptr)
  {
    std::cerr << "Error: Received null message in callback." << std::endl;
    return;
  }
  // Print the received image data
  std::cout << "Received Image data: ";
  std::cout << "Width: " << message->width() << ", ";
  std::cout << "Height: " << message->height() << ", ";
  std::cout << "Encoding: " << message->encoding() << std::endl;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // MessageType
  sensor_msgs::msg::ImageType sub_message_type;
  sensor_msgs::msg::ImageType pub_message_type;

  // Create a node with domain ID 0
  std::shared_ptr<Node> node = lwrcl::Node::make_shared("lwrcl_example");

  // Create a publisher with default QoS settings
  auto publisher_ptr = node->create_publisher<sensor_msgs::msg::Image>(&pub_message_type, "TESTTopic1", 10);
  if (publisher_ptr == nullptr)
  {
    std::cerr << "Error: Failed to create a publisher." << std::endl;
    return 1;
  }

  // Create a subscription with default QoS settings
  auto subscriber_ptr = node->create_subscription<sensor_msgs::msg::Image>(&sub_message_type, "TESTTopic2", 10, myCallbackFunction);
  if (subscriber_ptr == nullptr)
  {
    std::cerr << "Error: Failed to create a subscription." << std::endl;
    return 1;
  }

  int data_value = 0;
  rclcpp::Rate rate(Duration(std::chrono::milliseconds(100))); // Set rate to 100 milliseconds

  struct timespec curTime, lastTime;
  clock_gettime(CLOCK_REALTIME, &lastTime);

  // Main application loop
  while (ok())
  {
    // Check the number of publishers
    int32_t publisher_count = subscriber_ptr->get_publisher_count();
    std::cout << "Number of publishers: " << publisher_count << std::endl;

    clock_gettime(CLOCK_REALTIME, &curTime);
    // Print the interval between the last two messages
    if (curTime.tv_nsec < lastTime.tv_nsec)
    {
      printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec - 1, curTime.tv_nsec + 1000000000 - lastTime.tv_nsec);
    }
    else
    {
      printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec, curTime.tv_nsec - lastTime.tv_nsec);
    }
    lastTime = curTime;

    // Simulate sending data periodically
    sensor_msgs::msg::Image pub_message;
    pub_message.header().stamp().sec() = data_value;
    pub_message.header().stamp().nanosec() = data_value;
    pub_message.header().frame_id() = "TEST";
    pub_message.height() = 100;
    pub_message.width() = 200;
    pub_message.encoding() = "H263";
    pub_message.is_bigendian() = false;
    pub_message.step() = 1;
    pub_message.data() = {0, 0, 0, 0, 0, 0};

    // Publish the data
    publisher_ptr->publish(pub_message);

    // Handle incoming messages
    rclcpp::spin_some(node);

    LWRCL_WARN(node->get_logger(), "Publishing: '%s'", pub_message.encoding().c_str());

    data_value++;
    rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <chrono>

// lwrcl::Node::SharedPtr node = nullptr;

// int main(int argc, char * argv[]){
//   using namespace std::chrono_literals;

//   lwrcl::init(argc,argv);
//   node = lwrcl::Node::make_shared("minimal_node");

//   lwrcl::WallRate loop_rate(500ms);
//   for(int i=0 ; i<3 ; i++){
//     RCLCPP_INFO(node->get_logger(), "loop:%d",i);
//     loop_rate.sleep();
//   }

//   for(int i=0 ; i<3 ; i++){
//     RCLCPP_INFO(node->get_logger(), "loop:%d",i);
//     lwrcl::sleep_for(500ms);
//   }

//   auto timer1 = node->create_wall_timer(
//     1s,
//     [](){
//       RCLCPP_INFO(node->get_logger(),"node_loop");
//     }
//   );
//   lwrcl::spin(node);

//   lwrcl::shutdown();
//   return 0;
// }

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/qos.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <chrono>

// #ifndef STD_MSGS_MSG_STRINGTYPE_HPP
// #define STD_MSGS_MSG_STRINGTYPE_HPP
// FAST_DDS_DATA_TYPE(std_msgs, msg, String)
// #endif // STD_MSGS_MSG_STRINGTYPE_HPP

// int main(int argc, char * argv[]){
//   using namespace std::chrono_literals;
//   std_msgs::msg::StringType pub_message_type;

//   lwrcl::init(argc, argv);

//   auto node = lwrcl::Node::make_shared("minimal_publisher");
//   auto publisher = node->create_publisher<std_msgs::msg::String>(&pub_message_type,"topic_test",lwrcl::QoS(10));
//   auto message = std::make_shared<std_msgs::msg::String>();
//   auto pub_counter=0;

//   lwrcl::WallRate loop_rate(1s);

//   while(lwrcl::ok()){
//     message->data() = "hello " + std::to_string(pub_counter++);
//     RCLCPP_INFO(node->get_logger(),"Pub:%s",message->data().c_str());
//     publisher->publish(*message);
//     loop_rate.sleep();
//   }
//   lwrcl::shutdown();
//   return 0;
// }
