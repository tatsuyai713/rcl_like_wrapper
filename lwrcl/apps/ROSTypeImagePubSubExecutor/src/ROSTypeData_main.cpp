#include "ROSTypeImagePubSubMono.hpp"
#include "ROSTypeImagePubSubEdge.hpp"
#include <iostream>
#include <unistd.h>
#include <limits.h>
#include <string>

int main(int argc, char **argv)
{
  lwrcl::init(argc, argv);
  std::cout << "Starting ROSTypeImagePubSubExecutor" << std::endl;

  // Buffer to hold the path to the current executable
  char buf[PATH_MAX] = {0};
  std::string exePath, configPath;
  std::string configPath1, configPath2;

  // Attempt to read the symbolic link '/proc/self/exe' to find the path of the current executable
  ssize_t count = readlink("/proc/self/exe", buf, sizeof(buf));
  if (count != -1)
  {
    exePath = std::string(buf, count); // Convert to string and assign
    size_t pos = exePath.rfind('/');   // Find the last occurrence of '/'
    if (pos != std::string::npos)
    {
      configPath = exePath.substr(0, pos + 1); // Extract the path without executable name
    }
  }
  else
  {
    std::cerr << "Failed to determine the path of the executable." << std::endl;
    return 1;
  }

  // Initialize Executor
  lwrcl::executors::MultiThreadedExecutor executor;

  // Initialize and run the ROS-like node
  std::shared_ptr<ROSTypeImagePubSubMono> rcl_like_node1 = std::make_shared<ROSTypeImagePubSubMono>("ROSTypeImagePubSubExecutor");
  configPath1 = configPath + "config/config1.yaml"; // Append the relative path of the config file
  std::cout << "Using config file at: " << configPath1 << std::endl;

  if (rcl_like_node1->init_config(configPath1))
  {
    executor.add_node(rcl_like_node1);
  }
  else
  {
    std::cerr << "Failed to initialize the ROSTypeImagePubSubMono." << std::endl;
    return 1;
  }
  std::shared_ptr<ROSTypeImagePubSubEdge> rcl_like_node2 = std::make_shared<ROSTypeImagePubSubEdge>(rcl_like_node1->get_participant());
  configPath2 = configPath + "config/config2.yaml"; // Append the relative path of the config file
  std::cout << "Using config file at: " << configPath2 << std::endl;

  if (rcl_like_node2->init_config(configPath2))
  {
    executor.add_node(rcl_like_node2);
  }
  else
  {
    std::cerr << "Failed to initialize the ROSTypeImagePubSubEdge." << std::endl;
    return 1;
  }

  // Wait for the ROS-like node to finish
  executor.spin();
  executor.cancel();

  return 0;
}
