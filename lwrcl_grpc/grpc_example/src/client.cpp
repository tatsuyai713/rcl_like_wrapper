
#include <iostream>
#include <chrono>
#include <thread>
#include "grpc_rcl.hpp"
#include "std_msgs/msg/header.hpp"

// int main(int argc, char** argv) {
//     // Create a gRPC channel
//     grpc_rcl::Node node("localhost:50051");
//     std::string topic_name = "topic1";

//     std::shared_ptr<grpc_rcl::Publisher<std_msgs::msg::Header>> publisher = node.create_publisher<std_msgs::msg::Header>(topic_name);

//     // Create a Header message
//     std_msgs::msg::Header header;
//     header.frame_id("frame_1");
//     header.frame_id() = "frame_2";
//     header.stamp().sec(10);
//     header.stamp().nanosec() = 100;

//     // Send the message
//     int count = 0;
//     while (true) {
//         header.stamp().sec(count);
//         count++;
//         publisher->publish(header);
//         std::cout << "Message sent: " << header.frame_id() << std::endl;
//         std::cout << "Message sent: " << header.stamp().sec() << std::endl;
    
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
//     // publisher->publish(header);

//     node.spin();

//     return 0;
// }



int main() {
    // サーバーアドレスを指定
    const std::string server_address = "localhost:50051";

    // gRPC チャンネルを作成
    auto channel = grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());

    // Header クラスのインスタンスを作成
    std_msgs::msg::Header header;
    header.NewStub(channel);

    // メッセージデータを設定
    header.frame_id("example_frame");
    auto& stamp = header.stamp();
    stamp.nanosec(1627870123); // 例: 秒
    stamp.sec(123456789); // 例: ナノ秒

    // gRPC コンテキストを準備
    grpc::ClientContext context;

    // RPC メソッドを呼び出し
    grpc::Status status = header.send(context);

    // 結果を出力
    if (status.ok()) {
        std::cout << "Message sent successfully to server." << std::endl;
    } else {
        std::cerr << "Failed to send message. Error: " << status.error_message() << std::endl;
    }

    return 0;
}
