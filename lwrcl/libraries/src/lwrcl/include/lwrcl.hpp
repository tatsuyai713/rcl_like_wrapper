#ifndef LWRCL_HPP_
#define LWRCL_HPP_

#include <functional>
#include <string>
#include <unordered_map>
#include <forward_list>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <atomic>
#include <string>
#include <cstdarg> // For variable arguments handling
#include <iostream>
#include <chrono>
#include <iomanip>

#include "fast_dds_header.hpp"

#include "clock_time_duration.hpp"
#include "publisher.hpp"
#include "subscription.hpp"
#include "timer.hpp"

namespace lwrcl
{
  extern void lwrcl_signal_handler(int signal);

  class Logger;
  class Node;

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
    Parameter(const std::string &name, bool value) : name_(name), type_(Type::BOOL) {
        string_value_ = value ? "true" : "false";
    }

    // Constructor for int
    Parameter(const std::string &name, int value) : name_(name), type_(Type::INT) {
        string_value_ = std::to_string(value);
    }

    // Constructor for double
    Parameter(const std::string &name, double value) : name_(name), type_(Type::DOUBLE) {
        string_value_ = std::to_string(value);
    }

    // Constructor for std::string
    Parameter(const std::string &name, const std::string &value) : name_(name), type_(Type::STRING) {
        string_value_ = value;
    }

    // Constructor for const char*
    Parameter(const std::string &name, const char* value) : name_(name), type_(Type::STRING) {
        string_value_ = value;
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
    std::string as_string() const
    {
      return string_value_;
    }

  private:
    enum class Type
    {
      BOOL,
      INT,
      DOUBLE,
      STRING,
      UNKNOWN
    };

    std::string name_;
    std::string string_value_;
    Type type_;
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
      auto publisher = std::make_shared<Publisher<T>>(participant_.get(), std::string("rt/") + topic, depth);
      publisher_list_.push_front(publisher);
      return publisher;
    }

    template <typename T>
    std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic, const QoS &depth)
    {
      auto publisher = std::make_shared<Publisher<T>>(participant_.get(), std::string("rt/") + topic, depth.get_depth());
      publisher_list_.push_front(publisher);
      return publisher;
    }

    template <typename T>
    std::shared_ptr<Subscription<T>> create_subscription(const std::string &topic, const uint16_t &depth,
                                                         std::function<void(std::shared_ptr<T>)> callback_function)
    {
      auto subscription = std::make_shared<Subscription<T>>(participant_.get(), std::string("rt/") + topic, depth, callback_function, channel_);
      subscription_list_.push_front(subscription);
      return subscription;
    }

    template <typename T>
    std::shared_ptr<Subscription<T>> create_subscription(const std::string &topic, const QoS &depth,
                                                         std::function<void(std::shared_ptr<T>)> callback_function)
    {
      auto subscription = std::make_shared<Subscription<T>>(participant_.get(), std::string("rt/") + topic, depth.get_depth(), callback_function, channel_);
      subscription_list_.push_front(subscription);
      return subscription;
    }

    template <typename Rep, typename Period>
    std::shared_ptr<TimerBase> create_timer(std::chrono::duration<Rep, Period> period, std::function<void()> callback_function)
    {
      lwrcl::Clock::ClockType clock_type = Clock::ClockType::SYSTEM_TIME;
      auto duration = Duration(period);
      auto timer = std::make_shared<TimerBase>(duration, callback_function, channel_, clock_type);
      timer_list_.push_front(timer);
      return timer;
    }

    template <typename Rep, typename Period>
    std::shared_ptr<TimerBase> create_wall_timer(std::chrono::duration<Rep, Period> period, std::function<void()> callback_function)
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
    static std::shared_ptr<Node> make_shared(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
    static std::shared_ptr<Node> make_shared(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant, const std::string &name);

    friend void lwrcl::spin(std::shared_ptr<Node> node);
    friend void lwrcl::spin_some(std::shared_ptr<Node> node);

    virtual void shutdown();
    virtual Clock::SharedPtr get_clock();

    Node(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
    Node(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant, const std::string &name);
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

    void declare_parameter(const std::string &name, const char* default_value)
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
          eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant);
        }
      }
    };

    std::string get_params_file_path(int argc, char *argv[]);
    void loadParameters(const std::string &file_path);

    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;
    std::forward_list<std::shared_ptr<IPublisher>> publisher_list_;
    std::forward_list<std::shared_ptr<ISubscription>> subscription_list_;
    std::forward_list<std::shared_ptr<ITimerBase>> timer_list_;
    Channel<ChannelCallback *>::SharedPtr channel_;
    Clock::SharedPtr clock_;
    std::string name_;
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

} // namespace lwrcl

#define LWRCL_DEBUG(logger, ...) (logger).log(lwrcl::DEBUG, __VA_ARGS__)
#define LWRCL_INFO(logger, ...) (logger).log(lwrcl::INFO, __VA_ARGS__)
#define LWRCL_WARN(logger, ...) (logger).log(lwrcl::WARN, __VA_ARGS__)
#define LWRCL_ERROR(logger, ...) (logger).log(lwrcl::ERROR, __VA_ARGS__)

#endif // LWRCL_HPP_
