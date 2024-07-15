#include <csignal>
#include <cstring>
#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <algorithm>
#include <functional>
#include <memory>
#include <chrono>
#include <vector>
#include "lwrcl.hpp" // The main header file for the lwrcl namespace

namespace lwrcl
{
  class Node;

  class HandlerRegistry
  {
  public:
    std::vector<lwrcl::Node*> nodes_;
    std::mutex registry_mutex;

    void add_node(lwrcl::Node* node)
    {
      std::lock_guard<std::mutex> lock(registry_mutex);
      nodes_.emplace_back(node);
    }

    void remove_node(lwrcl::Node* node)
    {
      std::lock_guard<std::mutex> lock(registry_mutex);
      nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node), nodes_.end());
    }

    void notify_all()
    {
      std::lock_guard<std::mutex> lock(registry_mutex);
      for (auto node : nodes_)
      {
        if (node)
        {
          node->shutdown();
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }
    }
  };

  static HandlerRegistry &get_global_registry()
  {
    static HandlerRegistry global_registry;
    return global_registry;
  }

  // Global flag to control the stopping of the application, e.g., in response to SIGINT
  std::atomic_bool global_stop_flag{false};

  // Function to handle SIGINT signals for graceful application termination
  void lwrcl_signal_handler(int signal)
  {
    if (signal == SIGINT || signal == SIGTERM)
    {
      printf("SIGINT/SIGTERM received, shutting down...\n");
      global_stop_flag = true;
      get_global_registry().notify_all();
    }
  }

  void log(LogLevel level, const char *format, ...)
  {
    va_list args;
    va_start(args, format);

    std::ostringstream msg;
    auto now = std::chrono::system_clock::now();
    auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - now_sec);
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    char timestamp_buf[80];
    std::strftime(timestamp_buf, sizeof(timestamp_buf), "%s", std::localtime(&timestamp));

    switch (level)
    {
    case DEBUG:
      msg << "\033[0;36m[DEBUG]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case INFO:
      msg << "\033[0;37m[INFO]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case WARN:
      msg << "\033[0;33m[WARN]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case ERROR:
      msg << "\033[0;31m[ERROR]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    default:
      msg << "[Unknown log level] [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    }

    msg << ": ";
    std::cout << msg.str();
    vprintf(format, args);
    std::cout << std::endl;

    va_end(args);
  }

  Logger::Logger(const std::string &node_name) : node_name_(node_name) {}

  void Logger::log(LogLevel level, const char *format, ...) const
  {
    va_list args;
    va_start(args, format);

    std::ostringstream msg;
    auto now = std::chrono::system_clock::now();
    auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - now_sec);
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    char timestamp_buf[80];
    std::strftime(timestamp_buf, sizeof(timestamp_buf), "%s", std::localtime(&timestamp));

    switch (level)
    {
    case DEBUG:
      msg << "\033[0;36m[DEBUG]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case INFO:
      msg << "\033[0;37m[INFO]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case WARN:
      msg << "\033[0;33m[WARN]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case ERROR:
      msg << "\033[0;31m[ERROR]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    default:
      msg << "[Unknown log level] [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    }

    msg << "[" << node_name_ << "]: ";
    std::cout << msg.str();
    vprintf(format, args);
    std::cout << std::endl;

    va_end(args);
  }

  namespace executors
  {
    SingleThreadedExecutor::SingleThreadedExecutor() {}

    SingleThreadedExecutor::~SingleThreadedExecutor()
    {
      cancel();
    }

    void SingleThreadedExecutor::add_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.push_back(node);
      }
      else
      {
        std::cerr << "Error: Node pointer is null, cannot add to executor." << std::endl;
      }
    }

    void SingleThreadedExecutor::remove_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node), nodes_.end());
      }
    }

    void SingleThreadedExecutor::cancel()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto &node : nodes_)
      {
        if (node)
        {
          node->shutdown();
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }
    }

    void SingleThreadedExecutor::spin()
    {
      while (!global_stop_flag.load())
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto node : nodes_)
        {
          if (node)
          {
            lwrcl::spin_some(node);
          }
          else
          {
            std::cerr << "node pointer is invalid!" << std::endl;
          }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    }

    void SingleThreadedExecutor::spin_some()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        if (node)
        {
          lwrcl::spin_some(node);
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }
    }

    MultiThreadedExecutor::MultiThreadedExecutor() {}

    MultiThreadedExecutor::~MultiThreadedExecutor()
    {
      cancel();
    }

    void MultiThreadedExecutor::add_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.push_back(node);
      }
      else
      {
        std::cerr << "Error: Node pointer is null, cannot add to executor." << std::endl;
      }
    }

    void MultiThreadedExecutor::remove_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node), nodes_.end());
      }
    }

    void MultiThreadedExecutor::cancel()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        if (node)
        {
          node->shutdown();
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }

      for (auto &thread : threads_)
      {
        if (thread.joinable())
        {
          thread.join();
        }
      }
      threads_.clear();
    }

    void MultiThreadedExecutor::spin()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        threads_.emplace_back([this, node]()
                              {
          if (!node)
          {
            std::cerr << "node pointer is invalid!" << std::endl;
          }
          else
          {
            lwrcl::spin(node);
          } });
      }

      for (auto &thread : threads_)
      {
        if (thread.joinable())
        {
          thread.join();
        }
      }
    }

    void MultiThreadedExecutor::spin_some()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        if (node)
        {
          lwrcl::spin_some(node);
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }
    }

    int MultiThreadedExecutor::get_number_of_threads() const
    {
      std::lock_guard<std::mutex> lock(mutex_);
      return threads_.size();
    }
  } // namespace executors

  Time::Time() : nanoseconds_(0) {}
  Time::Time(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  Time::Time(int32_t seconds, uint32_t nanoseconds) : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds) {}
  int64_t Time::nanoseconds() const { return nanoseconds_; }
  double Time::seconds() const { return static_cast<double>(nanoseconds_) / 1e9; }
  Time Time::operator+(const Duration &rhs) const { return Time(nanoseconds_ + rhs.nanoseconds()); }
  Time Time::operator-(const Duration &rhs) const { return Time(nanoseconds_ - rhs.nanoseconds()); }
  Duration Time::operator-(const Time &rhs) const { return Duration(nanoseconds_ - rhs.nanoseconds_); }
  bool Time::operator==(const Time &rhs) const { return nanoseconds_ == rhs.nanoseconds_; }
  bool Time::operator!=(const Time &rhs) const { return !(*this == rhs); }
  bool Time::operator<(const Time &rhs) const { return nanoseconds_ < rhs.nanoseconds_; }
  bool Time::operator<=(const Time &rhs) const { return nanoseconds_ <= rhs.nanoseconds_; }
  bool Time::operator>(const Time &rhs) const { return nanoseconds_ > rhs.nanoseconds_; }
  bool Time::operator>=(const Time &rhs) const { return nanoseconds_ >= rhs.nanoseconds_; }

  Duration::Duration() : nanoseconds_(0) {}
  Duration::Duration(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  Duration::Duration(int32_t seconds, uint32_t nanoseconds) : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds) {}
  int64_t Duration::nanoseconds() const { return nanoseconds_; }
  double Duration::seconds() const { return static_cast<double>(nanoseconds_) / 1e9; }
  Duration Duration::operator+(const Duration &rhs) const { return Duration(nanoseconds_ + rhs.nanoseconds()); }
  Duration Duration::operator-(const Duration &rhs) const { return Duration(nanoseconds_ - rhs.nanoseconds()); }
  bool Duration::operator==(const Duration &rhs) const { return nanoseconds_ == rhs.nanoseconds_; }
  bool Duration::operator!=(const Duration &rhs) const { return !(*this == rhs); }
  bool Duration::operator<(const Duration &rhs) const { return nanoseconds_ < rhs.nanoseconds_; }
  bool Duration::operator<=(const Duration &rhs) const { return nanoseconds_ <= rhs.nanoseconds_; }
  bool Duration::operator>(const Duration &rhs) const { return nanoseconds_ > rhs.nanoseconds_; }
  bool Duration::operator>=(const Duration &rhs) const { return nanoseconds_ >= rhs.nanoseconds_; }

  // Clock implementation
  Clock::Clock(ClockType type) : type_(type) {}
  Time Clock::now()
  {
    switch (type_)
    {
    case ClockType::SYSTEM_TIME:
      return Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count());
    case ClockType::STEADY_TIME:
      return Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count());
    default:
      throw std::runtime_error("Unsupported clock type.");
    }
  }
  Clock::ClockType Clock::get_clock_type() const { return type_; }

  // Rate implementation
  Rate::Rate(const Duration &period) : period_(period), next_time_(std::chrono::system_clock::now() + std::chrono::nanoseconds(period.nanoseconds())) {}
  void Rate::sleep()
  {
    auto now = std::chrono::system_clock::now();
    if (now >= next_time_)
    {
      auto periods_missed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_time_) / std::chrono::nanoseconds(period_.nanoseconds()) + 1;
      next_time_ += periods_missed * std::chrono::nanoseconds(period_.nanoseconds());
    }
    std::this_thread::sleep_until(next_time_);
    next_time_ += std::chrono::nanoseconds(period_.nanoseconds());
  }

  WallRate::WallRate(const Duration &period) : period_(period), next_time_(std::chrono::steady_clock::now() + std::chrono::nanoseconds(period.nanoseconds())) {}
  void WallRate::sleep()
  {
    auto now = std::chrono::steady_clock::now();
    if (now >= next_time_)
    {
      auto periods_missed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_time_) / std::chrono::nanoseconds(period_.nanoseconds()) + 1;
      next_time_ += periods_missed * std::chrono::nanoseconds(period_.nanoseconds());
    }
    std::this_thread::sleep_until(next_time_);
    next_time_ += std::chrono::nanoseconds(period_.nanoseconds());
  }

  Node::Node(int domain_id) : clock_(std::make_unique<Clock>()), channel_(std::make_shared<Channel<ChannelCallback *>>())
  {
    name_ = "lwrcl_node"; // Default node name
    dds::DomainParticipantQos participant_qos = dds::PARTICIPANT_QOS_DEFAULT;

    // Create a descriptor for the new transport.
    auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
    udp_transport->sendBufferSize = 4194304;
    udp_transport->receiveBufferSize = 4194304;
    udp_transport->non_blocking_send = true;

    // Link the Transport Layer to the Participant.
    participant_qos.transport().user_transports.push_back(udp_transport);

    // Increase the sending buffer size
    participant_qos.transport().send_socket_buffer_size = 4194304;
    // Increase the receiving buffer size
    participant_qos.transport().listen_socket_buffer_size = 4194304;

    // eprosima::fastdds::dds::Log::SetVerbosity(eprosima::fastdds::dds::Log::Info);

    auto participant_factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
    participant_factory->load_XML_profiles_file("/opt/fast-dds/fastdds.xml");

    participant_ = std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>(
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(domain_id, participant_qos),
        DomainParticipantDeleter());
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
    get_global_registry().add_node(this);
  }

  Node::Node(int domain_id, const std::string &name) : clock_(std::make_unique<Clock>()), name_(name), channel_(std::make_shared<Channel<ChannelCallback *>>())
  {
    dds::DomainParticipantQos participant_qos = dds::PARTICIPANT_QOS_DEFAULT;

    // Create a descriptor for the new transport.
    auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
    udp_transport->sendBufferSize = 4194304;
    udp_transport->receiveBufferSize = 4194304;
    udp_transport->non_blocking_send = true;

    // Link the Transport Layer to the Participant.
    participant_qos.transport().user_transports.push_back(udp_transport);

    // Increase the sending buffer size
    participant_qos.transport().send_socket_buffer_size = 4194304;
    // Increase the receiving buffer size
    participant_qos.transport().listen_socket_buffer_size = 4194304;

    // eprosima::fastdds::dds::Log::SetVerbosity(eprosima::fastdds::dds::Log::Info);

    auto participant_factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
    participant_factory->load_XML_profiles_file("/opt/fast-dds/fastdds.xml");

    participant_ = std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>(
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(domain_id, participant_qos),
        DomainParticipantDeleter());
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
    get_global_registry().add_node(this);
  }

  Node::Node(const std::string &name) : clock_(std::make_unique<Clock>()), name_(name), channel_(std::make_shared<Channel<ChannelCallback *>>())
  {
    int domain_id = 0; // Default domain ID
    dds::DomainParticipantQos participant_qos = dds::PARTICIPANT_QOS_DEFAULT;

    // Create a descriptor for the new transport.
    auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
    udp_transport->sendBufferSize = 4194304;
    udp_transport->receiveBufferSize = 4194304;
    udp_transport->non_blocking_send = true;

    // Link the Transport Layer to the Participant.
    participant_qos.transport().user_transports.push_back(udp_transport);

    // Increase the sending buffer size
    participant_qos.transport().send_socket_buffer_size = 4194304;
    // Increase the receiving buffer size
    participant_qos.transport().listen_socket_buffer_size = 4194304;

    // eprosima::fastdds::dds::Log::SetVerbosity(eprosima::fastdds::dds::Log::Info);

    auto participant_factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
    participant_factory->load_XML_profiles_file("/opt/fast-dds/fastdds.xml");

    participant_ = std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>(
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(domain_id, participant_qos),
        DomainParticipantDeleter());
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
    get_global_registry().add_node(this);
  }

  Node::Node(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant) : clock_(std::make_unique<Clock>()), participant_(participant), channel_(std::make_shared<Channel<ChannelCallback *>>())
  {
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
    get_global_registry().add_node(this);
  }

  Node::~Node()
  {
    publisher_list_.clear();
    subscription_list_.clear();
    timer_list_.clear();
  }

  std::shared_ptr<Node> Node::make_shared(int domain_id)
  {
    auto node = std::shared_ptr<Node>(new Node(domain_id));
    get_global_registry().add_node(node.get());
    return node;
  }
  std::shared_ptr<Node> Node::make_shared(int domain_id, const std::string &name)
  {
    auto node = std::shared_ptr<Node>(new Node(domain_id, name));
    get_global_registry().add_node(node.get());
    return node;
  }
  std::shared_ptr<Node> Node::make_shared(const std::string &name)
  {
    auto node = std::shared_ptr<Node>(new Node(name));
    get_global_registry().add_node(node.get());
    return node;
  }
  std::shared_ptr<Node> Node::make_shared(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant)
  {
    auto node = std::shared_ptr<Node>(new Node(participant));
    get_global_registry().add_node(node.get());
    return node;
  }

  std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> Node::get_participant() const
  {
    return participant_;
  }

  std::string Node::get_name() const
  {
    return name_;
  }

  Logger Node::get_logger() const
  {
    return Logger(name_);
  }

  void Node::spin()
  {
    while (!channel_->is_closed() && !global_stop_flag.load())
    {
      ChannelCallback *callback;
      while (channel_->consume(callback))
      {
        if (callback)
        {
          callback->invoke();
        }
      }
    }
    printf("Node shutting down...\n");
    channel_->close();
  }

  void Node::spin_some()
  {
    bool event_processed = false;

    do
    {
      event_processed = false;

      ChannelCallback *callback;
      while (channel_->consume_nowait(callback))
      {
        callback->invoke();
        event_processed = true;
      }
    } while (event_processed);
  }

  void Node::shutdown()
  {
    channel_->close();
  }

  Clock::SharedPtr Node::get_clock()
  {
    return clock_;
  }

  QoS::QoS() : depth_(1) {}
  QoS::QoS(uint16_t depth) : depth_(depth) {}
  QoS::QoS(const QoS &qos) : depth_(qos.depth_) {}
  QoS::~QoS() {}
  QoS QoS::operator=(const QoS &qos)
  {
    depth_ = qos.depth_;
    return *this;
  }
  uint16_t QoS::get_depth() const { return depth_; }

  bool ok()
  {
    return !global_stop_flag.load();
  }

  void spin(std::shared_ptr<Node> node)
  {
    if (node)
    {
      node->spin();
    }
    else
    {
      std::cerr << "node pointer is invalid!" << std::endl;
    }
  }

  void spin_some(std::shared_ptr<Node> node)
  {
    if (node)
    {
      node->spin_some();
    }
    else
    {
      std::cerr << "node pointer is invalid!" << std::endl;
    }
  }

  void init(int argc, char *argv[])
  {
    // Do nothing;
    (void)argc;
    (void)argv;

    if (std::signal(SIGINT, lwrcl_signal_handler) == SIG_ERR || std::signal(SIGTERM, lwrcl_signal_handler) == SIG_ERR)
    {
      throw std::runtime_error("Failed to set signal handler.");
    }
  }

  void shutdown()
  {
    global_stop_flag = true;
    get_global_registry().notify_all();
  }

  void sleep_for(const lwrcl::Duration &duration)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(duration.nanoseconds()));
  }

} // namespace lwrcl
