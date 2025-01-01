#include "grpc_rcl.hpp"
#include "std_msgs/msg/header.hpp"

void callback(std_msgs::msg::Header &header)
{
    std::cout << "Callback: Processing Header with frame_id: " << header.frame_id() << std::endl;
    std::cout << "Callback: Processing Header with stamp.sec: " << header.stamp().sec() << std::endl;
    // ここにコールバック処理を実装
}

int main(int argc, char **argv)
{
    // Create and start the gRPC server
    std::string topic_name = "topic1";
    grpc_rcl::Node node("0.0.0.0:50051");

    // Create a subscription
    std::shared_ptr<grpc_rcl::Subscription<std_msgs::msg::Header>> subscription = node.create_subscription<std_msgs::msg::Header>(topic_name, callback);

    node.spin();
    return 0;
}
