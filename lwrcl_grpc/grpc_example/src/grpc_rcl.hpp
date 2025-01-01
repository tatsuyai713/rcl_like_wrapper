#ifndef GRPC_NODE_HPP_
#define GRPC_NODE_HPP_

#include <grpcpp/grpcpp.h>
#include <string>
#include <memory>
#include <unordered_map>
#include <set>
#include <regex>
#include <google/protobuf/empty.pb.h>


namespace grpc_rcl {

void init(int& argc, char**& argv) {
    // Initialize gRPC library if needed
}

void shutdown() {
    // Shutdown gRPC library if needed
}

// ======================================== //
// Define the Publisher class
template<typename T>
class Publisher {
public:
    // コンストラクタでトピック名を受け取る
    Publisher(std::shared_ptr<grpc::Channel> channel, const std::string& topic_name)
        : channel_(channel), topic_name_(topic_name) 
        {

        }

    void publish(T& message) 
    {
        grpc::ClientContext context;
        // サーバーにトピック名をメタデータとして設定する
        context.AddMetadata("topic-name", topic_name_);

        message.NewStub(channel_);
        grpc::Status status = message.send(context);

        if (status.ok()) {
            std::cout << "Message sent successfully on topic: " << topic_name_ << std::endl;
        } else {
            std::cout << "Error: " << status.error_message() << std::endl;
        }
    }

private:
    std::shared_ptr<grpc::Channel> channel_;
    std::string topic_name_;  // クライアント側で使用するトピック名
};

// ======================================== //

// クライアント情報を管理するクラス
class ClientManager
{
public:
    // クライアントのIPと許可されたトピック名のセットを登録
    void register_client(const std::string &client_ip, const std::string &topic_name)
    {
        client_topics_[client_ip] = topic_name;
    }

    // クライアントのIPに対してトピック名をチェック
    bool is_client_allowed(const std::string &client_ip, const std::string &topic_name) const
    {
        auto it = client_topics_.find(client_ip);
        return it != client_topics_.end() && it->second == topic_name;
    }

private:
    std::unordered_map<std::string, std::string> client_topics_; // クライアントIPとトピック名の対応
};

std::string url_decode(const std::string &encoded)
{
    std::string decoded;
    char ch;
    int i, len = encoded.length();

    for (i = 0; i < len; i++)
    {
        if (encoded[i] == '%')
        {
            int val;
            std::istringstream hex(encoded.substr(i + 1, 2));
            if (hex >> std::hex >> val)
            {
                decoded += static_cast<char>(val);
                i += 2;
            }
        }
        else
        {
            decoded += encoded[i];
        }
    }
    return decoded;
}

// クライアントIPを正しくパースする関数
std::string parse_ip(const std::string &peer)
{
    // まずはURLデコードを実施
    std::string decoded_peer = url_decode(peer);

    // IPv6アドレスをパースする正規表現
    std::regex ipv6_regex(R"(ipv6:\[(.*)\]:\d+)");
    // IPv4アドレスをパースする正規表現
    std::regex ipv4_regex(R"(ipv4:(\d+\.\d+\.\d+\.\d+):\d+)");
    std::smatch match;

    // IPv6アドレスをチェック
    if (std::regex_search(decoded_peer, match, ipv6_regex))
    {
        return match[1].str(); // IPv6アドレス部分を返す
    }
    // IPv4アドレスをチェック
    else if (std::regex_search(decoded_peer, match, ipv4_regex))
    {
        return match[1].str(); // IPv4アドレス部分を返す
    }

    return decoded_peer; // パースできなかった場合は元の文字列を返す
}

// Implement the HeaderService subscription
template <typename T>
class Subscription final : public T
{
public:
    using CallbackType = std::function<void(T &)>;

    // コンストラクタでコールバックとクライアントマネージャーを受け取る
    Subscription(const std::string &topic_name, CallbackType callback)
        : topic_name_(topic_name), callback_(std::move(callback))
    {
        client_manager_.register_client("::1", topic_name_);       // ローカルホストのクライアントには"topic1"を許可 (IPv6)
        client_manager_.register_client("127.0.0.1", topic_name_); // ローカルホストのクライアントには"topic1"を許可 (IPv4)
    }

    // gRPCリクエストが来た際にクライアントIPをチェックし、コールバックを実行
    grpc::Status SendGRPC(grpc::ServerContext *context, const decltype(T::type_) *request, google::protobuf::Empty *response) override
    {

        std::string client_ip = parse_ip(context->peer()); // クライアントのIPアドレスを正しく解析
        std::cout << "Parsed Client IP: " << client_ip << std::endl;

        // メタデータからトピック名を取得
        auto metadata = context->client_metadata();
        auto topic_iter = metadata.find("topic-name");

        if (topic_iter == metadata.end())
        {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Topic name not provided");
        }

        std::string topic_name(topic_iter->second.data(), topic_iter->second.length());

        // クライアントが許可されているか確認
        if (!client_manager_.is_client_allowed(client_ip, topic_name))
        {
            std::cout << "Client is not allowed for topic: " << topic_name << std::endl;
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, "Client not allowed for this topic");
        }

        std::cout << "Client allowed for topic: " << topic_name << std::endl;

        // コールバックが設定されていれば呼び出す
        if (callback_)
        {
            decltype(T::type_) message_grpc(*request);
            T message(&message_grpc);
            // message.grpc_ = new decltype(T::type_)(*request);
            callback_(message);
            // delete message.grpc_;
        }

        return grpc::Status::OK;
    }

private:
    CallbackType callback_;
    ClientManager client_manager_; // クライアント情報を管理するマネージャー
    std::string topic_name_;       // 許可されたトピック名
};



// ======================================== //
class Node {
public:
    Node(const std::string& server_address)
        : server_address_(server_address) 
    {
        builder_.AddListeningPort(server_address_, grpc::InsecureServerCredentials());
    }

    template<typename T>
    std::shared_ptr<Publisher<T>> create_publisher(const std::string& topic_name) 
    {
        return std::make_shared<grpc_rcl::Publisher<T>>(grpc::CreateChannel(server_address_, grpc::InsecureChannelCredentials()), topic_name);
    }

    template<typename T>
    std::shared_ptr<Subscription<T>> create_subscription(const std::string& topic_name, std::function<void(T&)> callback) 
    {
        std::shared_ptr<Subscription<T>> subscription = std::make_shared<grpc_rcl::Subscription<T>>(topic_name, callback);
        builder_.RegisterService(subscription.get());
        return subscription;
    }

    ~Node() {
        server_->Shutdown();
    }
    void spin() {
        server_ = builder_.BuildAndStart();
        server_->Wait();
    }

private:
    std::string server_address_;
    std::shared_ptr<grpc::Server> server_;
    grpc::ServerBuilder builder_;
};

}  // namespace grpc_rcl

#endif  // GRPC_NODE_HPP_
