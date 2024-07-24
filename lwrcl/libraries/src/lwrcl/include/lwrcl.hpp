#ifndef LWRCL_HPP_
#define LWRCL_HPP_

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdarg> // For variable arguments handling
#include <cstring>
#include <forward_list>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "clock_time_duration.hpp"
#include "fast_dds_header.hpp"
#include "publisher.hpp"
#include "subscription.hpp"
#include "timer.hpp"

namespace lwrcl
{
  extern void lwrcl_signal_handler(int signal);

  class Logger;
  class QoS;
  class Node;
  class IService;
  class IClient;
  template <typename Req, typename Res>
  class Service;
  template <typename Req, typename Res>
  class Client;

  // lwrcl functions
  bool ok(void);
  void spin(std::shared_ptr<lwrcl::Node> node);
  void init(int argc, char *argv[]);
  void shutdown(void);
  void sleep_for(const lwrcl::Duration &duration);
  void spin_some(std::shared_ptr<lwrcl::Node> node);

  // Base class for different parameter types

  class ParameterBase
  {
  public:
    virtual ~ParameterBase() = default;
    virtual std::string get_name() const = 0;
    virtual std::string as_string() const = 0;
  };

  class Parameter : public ParameterBase
  {
  public:
    // Constructor for bool
    Parameter(const std::string &name, bool value) : name_(name), type_(Type::BOOL)
    {
      string_value_ = value ? "true" : "false";
    }

    // Constructor for int
    Parameter(const std::string &name, int value) : name_(name), type_(Type::INT)
    {
      string_value_ = int_to_string(value);
    }

    // Constructor for double
    Parameter(const std::string &name, double value) : name_(name), type_(Type::DOUBLE)
    {
      string_value_ = double_to_string(value);
    }

    // Constructor for std::string
    Parameter(const std::string &name, const std::string &value) : name_(name), type_(Type::STRING)
    {
      string_value_ = value;
    }

    // Constructor for const char*
    Parameter(const std::string &name, const char *value) : name_(name), type_(Type::STRING)
    {
      string_value_ = value;
    }

    // Constructor for bool array
    Parameter(const std::string &name, const std::vector<bool> &value)
        : name_(name), type_(Type::BOOL_ARRAY)
    {
      string_value_ = vector_to_string(value);
    }

    // Constructor for int array
    Parameter(const std::string &name, const std::vector<int> &value)
        : name_(name), type_(Type::INT_ARRAY)
    {
      string_value_ = vector_to_string(value);
    }

    // Constructor for double array
    Parameter(const std::string &name, const std::vector<double> &value)
        : name_(name), type_(Type::DOUBLE_ARRAY)
    {
      string_value_ = vector_to_string(value);
    }

    // Constructor for std::string array
    Parameter(const std::string &name, const std::vector<std::string> &value)
        : name_(name), type_(Type::STRING_ARRAY)
    {
      string_value_ = vector_to_string(value);
    }

    // Constructor for const char* array
    Parameter(const std::string &name, std::vector<const char *> &value)
        : name_(name), type_(Type::STRING_ARRAY)
    {
      string_value_ = vector_to_string(value);
    }

    // Constructor for Byte array
    Parameter(const std::string &name, const std::vector<uint8_t> &value)
        : name_(name), type_(Type::BYTE_ARRAY)
    {
      string_value_ = vector_to_string(value);
    }

    Parameter() : type_(Type::UNKNOWN) {}

    // Get name
    std::string get_name() const override { return name_; }

    // Get value (specific to type)
    bool as_bool() const
    {
      if (type_ != Type::BOOL)
      {
        throw std::runtime_error("Parameter is not a bool");
      }
      return string_value_ == "true";
    }

    int as_int() const
    {
      if (type_ != Type::INT)
      {
        throw std::runtime_error("Parameter is not an int");
      }
      int int_value;
      std::istringstream iss(string_value_);
      iss >> int_value;
      return int_value;
    }
    double as_double() const
    {
      if (type_ != Type::DOUBLE)
      {
        throw std::runtime_error("Parameter is not a double");
      }
      double double_value;
      std::istringstream iss(string_value_);
      iss >> double_value;
      return double_value;
    }
    std::string as_string() const { return string_value_; }

    std::vector<bool> as_bool_array() const
    {
      if (type_ != Type::BOOL_ARRAY)
      {
        throw std::runtime_error("Parameter is not a bool array");
      }
      return string_to_vector<bool>(string_value_);
    }

    std::vector<int> as_integer_array() const
    {
      if (type_ != Type::INT_ARRAY)
      {
        throw std::runtime_error("Parameter is not an int array");
      }
      return string_to_vector<int>(string_value_);
    }

    std::vector<double> as_double_array() const
    {
      if (type_ != Type::DOUBLE_ARRAY)
      {
        throw std::runtime_error("Parameter is not a double array");
      }
      return string_to_vector<double>(string_value_);
    }

    std::vector<std::string> as_string_array() const
    {
      if (type_ != Type::STRING_ARRAY)
      {
        throw std::runtime_error("Parameter is not a string array");
      }
      return string_to_vector<std::string>(string_value_);
    }

    std::vector<uint8_t> as_byte_array() const
    {
      if (type_ != Type::BYTE_ARRAY)
      {
        throw std::runtime_error("Parameter is not a string array");
      }
      return string_to_vector<uint8_t>(string_value_);
    }

  private:
    enum class Type
    {
      BOOL,
      INT,
      DOUBLE,
      STRING,
      BOOL_ARRAY,
      INT_ARRAY,
      DOUBLE_ARRAY,
      STRING_ARRAY,
      BYTE_ARRAY,
      UNKNOWN
    };

    std::string name_;
    std::string string_value_;
    Type type_;

    // Convert int to string
    static std::string int_to_string(int value)
    {
      std::ostringstream oss;
      oss << value;
      return oss.str();
    }

    // Convert double to string
    static std::string double_to_string(double value)
    {
      std::ostringstream oss;
      oss << value;
      return oss.str();
    }

    // Convert vector to string
    template <typename T>
    static std::string vector_to_string(const std::vector<T> &vec)
    {
      std::ostringstream oss;
      for (size_t i = 0; i < vec.size(); ++i)
      {
        if (i > 0)
        {
          oss << ",";
        }
        oss << vec[i];
      }
      return oss.str();
    }

    // Convert string to vector
    template <typename T>
    static std::vector<T> string_to_vector(const std::string &str)
    {
      std::vector<T> vec;
      std::istringstream iss(str);
      std::string item;
      while (std::getline(iss, item, ','))
      {
        std::istringstream converter(item);
        T value;
        converter >> value;
        vec.push_back(value);
      }
      return vec;
    }
  };

  class QoS
  {
  public:
    QoS();
    QoS(uint16_t depth);
    QoS(const QoS &qos);
    QoS operator=(const QoS &qos);
    ~QoS();
    uint16_t get_depth() const;

  private:
    uint16_t depth_;
  };

  typedef std::unordered_map<std::string, Parameter> Parameters;
  typedef std::unordered_map<std::string, Parameters> NodeParameters;

  extern NodeParameters node_parameters;

  class Node : public std::enable_shared_from_this<Node>
  {
  public:
    using SharedPtr = std::shared_ptr<Node>;

    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> get_participant() const;
    std::string get_name() const;
    Logger get_logger() const;

    template <typename T>
    std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic, const uint16_t &depth)
    {
      auto publisher =
          std::make_shared<Publisher<T>>(participant_.get(), std::string("rt/") + topic, depth);
      publisher_list_.push_front(publisher);
      return publisher;
    }

    template <typename T>
    std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic, const QoS &depth)
    {
      auto publisher = std::make_shared<Publisher<T>>(
          participant_.get(), std::string("rt/") + topic, depth.get_depth());
      publisher_list_.push_front(publisher);
      return publisher;
    }

    template <typename T>
    std::shared_ptr<Subscription<T>> create_subscription(
        const std::string &topic, const uint16_t &depth,
        std::function<void(std::shared_ptr<T>)> callback_function)
    {
      auto subscription = std::make_shared<Subscription<T>>(
          participant_.get(), std::string("rt/") + topic, depth, callback_function, channel_);
      subscription_list_.push_front(subscription);
      return subscription;
    }

    template <typename T>
    std::shared_ptr<Subscription<T>> create_subscription(
        const std::string &topic, const QoS &depth,
        std::function<void(std::shared_ptr<T>)> callback_function)
    {
      auto subscription = std::make_shared<Subscription<T>>(
          participant_.get(), std::string("rt/") + topic, depth.get_depth(), callback_function,
          channel_);
      subscription_list_.push_front(subscription);
      return subscription;
    }

    template <typename Req, typename Res>
    std::shared_ptr<Service<Req, Res>> create_service(
        const std::string &service_name,
        std::function<void(std::shared_ptr<Req>, std::shared_ptr<Res>)> callback_function)
    {
      std::shared_ptr<Service<Req, Res>> service =
          std::make_shared<Service<Req, Res>>(participant_, service_name, callback_function, channel_);
      service_list_.push_front(service);

      return service;
    }

    template <typename Req, typename Res>
    std::shared_ptr<Client<Req, Res>> create_client(const std::string &service_name)
    {
      std::shared_ptr<Client<Req, Res>> client =
          std::make_shared<Client<Req, Res>>(participant_, service_name, channel_);
      client_list_.push_front(client);

      return client;
    }

    template <typename Rep, typename Period>
    std::shared_ptr<TimerBase> create_timer(
        std::chrono::duration<Rep, Period> period, std::function<void()> callback_function)
    {
      lwrcl::Clock::ClockType clock_type = Clock::ClockType::SYSTEM_TIME;
      auto duration = Duration(period);
      auto timer = std::make_shared<TimerBase>(duration, callback_function, channel_, clock_type);
      timer_list_.push_front(timer);
      return timer;
    }

    template <typename Rep, typename Period>
    std::shared_ptr<TimerBase> create_wall_timer(
        std::chrono::duration<Rep, Period> period, std::function<void()> callback_function)
    {
      lwrcl::Clock::ClockType clock_type = Clock::ClockType::STEADY_TIME;
      auto duration = Duration(period);
      auto timer = std::make_shared<TimerBase>(duration, callback_function, channel_, clock_type);
      timer_list_.push_front(timer);
      return timer;
    }

    static std::shared_ptr<Node> make_shared(int domain_id);
    static std::shared_ptr<Node> make_shared(int domain_id, const std::string &name);
    static std::shared_ptr<Node> make_shared(const std::string &name);
    static std::shared_ptr<Node> make_shared(
        std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
    static std::shared_ptr<Node> make_shared(
        std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant,
        const std::string &name);

    friend void lwrcl::spin(std::shared_ptr<Node> node);
    friend void lwrcl::spin_some(std::shared_ptr<Node> node);

    virtual void shutdown();
    virtual Clock::SharedPtr get_clock();

    Node(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
    Node(
        std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant,
        const std::string &name);
    virtual ~Node();

    void set_parameters(const std::vector<std::shared_ptr<ParameterBase>> &parameters)
    {
      for (const auto &param : parameters)
      {
        std::string node_name = this->get_name();
        std::string param_name = param->get_name();

        // Check if the node exists in node_parameters
        auto node_it = node_parameters.find(node_name);
        if (node_it != node_parameters.end())
        {
          Parameters &params = node_it->second;

          // Check if the parameter exists in the node_parameters for this node
          if (params.find(param_name) != params.end())
          {
            // Update the existing parameter
            params[param_name] = *std::dynamic_pointer_cast<Parameter>(param);
            parameters_[param_name] = *std::dynamic_pointer_cast<Parameter>(param);

            std::cout << "Parameter updated: " << param_name << std::endl;
          }
          else
          {
            std::cerr << "Parameter not found: " << param_name << std::endl;
          }
        }
        else
        {
          std::cerr << "Node not found: " << node_name << std::endl;
        }
      }
    }

    void set_parameters(const std::vector<Parameter> &parameters)
    {
      std::vector<std::shared_ptr<ParameterBase>> base_params;
      for (const auto &param : parameters)
      {
        base_params.push_back(std::make_shared<Parameter>(param));
      }
      set_parameters(base_params);
    }

    void declare_parameter(const std::string &name, const bool &default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    void declare_parameter(const std::string &name, const int &default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    void declare_parameter(const std::string &name, const double &default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    void declare_parameter(const std::string &name, const std::string &default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    void declare_parameter(const std::string &name, const char *default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    void declare_parameter(const std::string &name, const std::vector<bool> default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    void declare_parameter(const std::string &name, const std::vector<int> default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    void declare_parameter(const std::string &name, const std::vector<double> default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    void declare_parameter(const std::string &name, const std::vector<std::string> default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    void declare_parameter(const std::string &name, const std::vector<uint8_t> default_value)
    {
      std::string node_name = this->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        const Parameters &params = node_it->second;
        auto param_it = params.find(name);
        if (param_it != params.end())
        {
          Parameter param_value = param_it->second;
          parameters_[name] = param_value;
          node_parameters[node_name][name] = param_value;
          return;
        }
      }

      parameters_[name] = Parameter(name, default_value);
      node_parameters[node_name][name] = Parameter(name, default_value);
    }

    Parameter get_parameter(const std::string &name) const
    {
      auto it = parameters_.find(name);
      if (it != parameters_.end())
      {
        return it->second;
      }
      else
      {
        throw std::runtime_error("Parameter not found");
      }
    }

    void get_parameter(const std::string &name, bool &bool_data) const
    {
      auto it = parameters_.find(name);
      if (it != parameters_.end())
      {
        Parameter param = it->second;
        bool_data = param.as_bool();
      }
      else
      {
        throw std::runtime_error("Parameter not found");
      }
    }

    void get_parameter(const std::string &name, int &int_data) const
    {
      auto it = parameters_.find(name);
      if (it != parameters_.end())
      {
        Parameter param = it->second;
        int_data = param.as_int();
      }
      else
      {
        throw std::runtime_error("Parameter not found");
      }
    }

    void get_parameter(const std::string &name, double &double_data) const
    {
      auto it = parameters_.find(name);
      if (it != parameters_.end())
      {
        Parameter param = it->second;
        double_data = param.as_double();
      }
      else
      {
        throw std::runtime_error("Parameter not found");
      }
    }

    void get_parameter(const std::string &name, std::string &string_data) const
    {
      auto it = parameters_.find(name);
      if (it != parameters_.end())
      {
        Parameter param = it->second;
        string_data = param.as_string();
      }
      else
      {
        throw std::runtime_error("Parameter not found");
      }
    }

    int closed_;
    void stop_spin();
    bool stop_flag_;

  private:
    virtual void spin();
    virtual void spin_some();

  protected:
    Node(int domain_id);
    Node(int domain_id, const std::string &name);
    Node(const std::string &name);

  private:
    struct DomainParticipantDeleter
    {
      void operator()(eprosima::fastdds::dds::DomainParticipant *participant) const
      {
        if (participant != nullptr)
        {
          eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(
              participant);
        }
      }
    };

    std::string get_params_file_path(int argc, char *argv[]);
    void loadParameters(const std::string &file_path);

    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;
    Channel<ChannelCallback *>::SharedPtr channel_;
    Clock::SharedPtr clock_;
    std::string name_;
    std::forward_list<std::shared_ptr<IPublisher>> publisher_list_;
    std::forward_list<std::shared_ptr<ISubscription>> subscription_list_;
    std::forward_list<std::shared_ptr<ITimerBase>> timer_list_;
    std::forward_list<std::shared_ptr<IService>> service_list_;
    std::forward_list<std::shared_ptr<IClient>> client_list_;
    Parameters parameters_;
  };

  namespace executors
  {
    // Executor that manages and executes nodes in a single thread.
    class SingleThreadedExecutor
    {
    public:
      SingleThreadedExecutor();
      ~SingleThreadedExecutor();

      void add_node(Node::SharedPtr node);
      void remove_node(Node::SharedPtr node);
      void cancel();
      void spin();
      void spin_some();
      // void spin_once(std::chrono::nanoseconds timeout);

    private:
      std::vector<Node::SharedPtr> nodes_; // List of nodes managed by the executor.
      std::mutex mutex_;                   // Mutex for thread-safe access to the nodes list.
    };

    // Executor that manages and executes nodes, each in its own thread, allowing for parallel processing.
    class MultiThreadedExecutor
    {
    public:
      MultiThreadedExecutor();
      ~MultiThreadedExecutor();

      void add_node(Node::SharedPtr node);
      void remove_node(Node::SharedPtr node);
      void cancel();
      void spin();
      void spin_some();
      // void spin_once(std::chrono::nanoseconds timeout);
      int get_number_of_threads() const;

    private:
      std::vector<Node::SharedPtr> nodes_; // List of nodes managed by the executor.
      std::vector<std::thread> threads_;   // Threads created for each node for parallel execution.
      mutable std::mutex mutex_;           // Mutex for thread-safe access to the nodes and threads lists.
    };
  } // namespace executors

  class Rate
  {
  private:
    Duration period_;
    std::chrono::system_clock::time_point next_time_;

  public:
    explicit Rate(const Duration &period);
    void sleep();
  };

  class WallRate
  {
  private:
    Duration period_;
    std::chrono::steady_clock::time_point next_time_;

  public:
    explicit WallRate(const Duration &period);
    void sleep();
  };

  enum LogLevel
  {
    DEBUG,
    INFO,
    WARN,
    ERROR
  };

  void log(LogLevel level, const char *format, ...);

  // Logger クラス
  class Logger
  {
  public:
    Logger(const std::string &node_name);

    void log(LogLevel level, const char *format, ...) const;

  private:
    std::string node_name_;
  };

  class IService
  {
  public:
    virtual ~IService() = default;
    virtual void stop() = 0;
  };

  template <typename Req, typename Res>
  class Service : public IService, public std::enable_shared_from_this<Service<Req, Res>>
  {
  public:
    using SharedPtr = std::shared_ptr<Service>;

    Service(
        std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant,
        const std::string &service_name,
        std::function<void(std::shared_ptr<Req>, std::shared_ptr<Res>)> callback_function,
        std::shared_ptr<Channel<ChannelCallback *>> channel)
        : participant_(participant), service_name_(service_name), callback_function_(callback_function), channel_(channel)
    {

      // Create topic
      request_topic_name_ = service_name_ + "_Request";
      response_topic_name_ = service_name_ + "_Response";

      publisher_ = std::make_shared<Publisher<Res>>(
          participant_.get(), std::string("rp/") + response_topic_name_, 1);

      request_callback_function_ = [this](std::shared_ptr<Req> request)
      {
        std::shared_ptr<Res> response = std::make_shared<Res>();
        callback_function_(request, response);
        publisher_->publish(response);
      };

      subscription_ = std::make_shared<Subscription<Req>>(
          participant_.get(), std::string("rp/") + request_topic_name_, 1, request_callback_function_,
          channel_);
    }

    ~Service() {} // Destructor

    void stop() override { subscription_->stop(); }

  private:
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;
    std::string service_name_;
    std::function<void(std::shared_ptr<Req>, std::shared_ptr<Res>)> callback_function_;
    std::function<void(std::shared_ptr<Req>)> request_callback_function_;
    std::shared_ptr<Publisher<Res>> publisher_;
    std::shared_ptr<Subscription<Req>> subscription_;
    std::string request_topic_name_;
    std::string response_topic_name_;
    Channel<ChannelCallback *>::SharedPtr channel_;
  };

  class IClient
  {
  public:
    virtual ~IClient() = default;
    virtual void stop() = 0;
  };

  enum FutureReturnCode
  {
    SUCCESS,
    INTERRUPTED,
    TIMEOUT
  };

  class FutureBase
  {
  public:
    virtual ~FutureBase() = default;
    virtual std::future_status wait_for(std::chrono::milliseconds timeout) = 0;
  };

  template <typename Res>
  class TypedFuture : public FutureBase
  {
  public:
    TypedFuture(std::shared_future<std::shared_ptr<Res>> future)
        : future_(future) {}

    std::future_status wait_for(std::chrono::milliseconds timeout) override
    {
      return future_.wait_for(timeout);
    }

  private:
    std::shared_future<std::shared_ptr<Res>> future_;
  };

  template <typename Req, typename Res>
  class Client : public IClient, public std::enable_shared_from_this<Client<Req, Res>>
  {
  public:
    using SharedPtr = std::shared_ptr<Client>;

    Client(
        std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant,
        const std::string &service_name,
        std::shared_ptr<Channel<ChannelCallback *>> channel)
        : participant_(participant),
          service_name_(service_name),
          channel_(channel),
          response_(nullptr),
          response_callback_function_(nullptr)
    {

      // Create topic
      request_topic_name_ = service_name_ + "_Request";
      response_topic_name_ = service_name_ + "_Response";

      publisher_ = std::make_shared<Publisher<Req>>(
          participant_.get(), std::string("rp/") + request_topic_name_, 1);

      subscription_ = std::make_shared<Subscription<Res>>(
          participant_.get(), std::string("rp/") + response_topic_name_, 1,
          std::function<void(std::shared_ptr<Res>)>(
              std::bind(&Client::handle_response, this, std::placeholders::_1)),
          channel_);
    }
    ~Client()
    {
    } // Destructor

    void handle_response(std::shared_ptr<Res> response)
    {
      if (response_callback_function_)
      {
        response_callback_function_(response);
      }
      std::lock_guard<std::mutex> lock(mutex_);
      response_ = response;
      response_received_ = true;

      cv_.notify_one();
    }

    void stop() override { subscription_->stop(); }

    std::shared_ptr<FutureBase> async_send_request(std::shared_ptr<Req> request)
    {
      auto promise = std::make_shared<std::promise<std::shared_ptr<Res>>>();
      auto future = promise->get_future().share();

      response_callback_function_ = [promise](std::shared_ptr<void> response) mutable
      {
        promise->set_value(std::static_pointer_cast<Res>(response));
      };

      publisher_->publish(request);

      return std::make_shared<TypedFuture<Res>>(future);
    }

    // void spin() {

    //   while (!response_received_)
    //   {
    //     ChannelCallback *callback;
    //     while (channel_->consume(callback))
    //     {
    //       if (callback)
    //       {
    //         callback->invoke();
    //       }
    //       else
    //       {
    //         break;
    //       }
    //     }
    //   }
    // }

    template <typename Duration>
    bool wait_for_service(const Duration &timeout)
    {
      std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
      std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
      std::chrono::system_clock::time_point end_time = start_time + timeout;

      while (current_time < end_time)
      {
        if (publisher_->get_subscriber_count() > 0)
        {
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        current_time = std::chrono::system_clock::now();
      }

      return false;
    }

  private:
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;
    std::string service_name_;
    std::shared_ptr<Publisher<Req>> publisher_;
    std::shared_ptr<Subscription<Res>> subscription_;
    std::string request_topic_name_;
    std::string response_topic_name_;
    std::function<void(std::shared_ptr<Res>)> response_callback_function_;
    std::shared_ptr<Res> response_;
    Channel<ChannelCallback *>::SharedPtr channel_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool response_received_ = false;
  }; //

  template <typename Duration>
  FutureReturnCode spin_until_future_complete(
      std::shared_ptr<lwrcl::Node> node,
      std::shared_ptr<FutureBase> future,
      const Duration &timeout)
  {
    std::thread spin_thread([node]()
                            { lwrcl::spin(node); });

    if (future->wait_for(std::chrono::duration_cast<std::chrono::milliseconds>(timeout)) == std::future_status::ready)
    {
      node->stop_spin();
      spin_thread.join();
      return SUCCESS;
    }
    else
    {
      node->stop_spin();
      spin_thread.join();
      return TIMEOUT;
    }
  }
} // namespace lwrcl

#define LWRCL_DEBUG(logger, ...) (logger).log(lwrcl::DEBUG, __VA_ARGS__)
#define LWRCL_INFO(logger, ...) (logger).log(lwrcl::INFO, __VA_ARGS__)
#define LWRCL_WARN(logger, ...) (logger).log(lwrcl::WARN, __VA_ARGS__)
#define LWRCL_ERROR(logger, ...) (logger).log(lwrcl::ERROR, __VA_ARGS__)

#endif // LWRCL_HPP_
