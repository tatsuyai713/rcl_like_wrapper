#include <iostream> // 追加

#ifndef LWRCL_SUBSCRIBER_HPP_
#define LWRCL_SUBSCRIBER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fast_dds_header.hpp"
#include "qos.hpp"
#include "channel.hpp"

namespace lwrcl
{
  template <typename T>
  class SubscriberCallback : public ChannelCallback
  {
  public:
    SubscriberCallback(std::function<void(std::shared_ptr<T>)> callback_function, std::vector<std::shared_ptr<T>> *message_buffer)
        : callback_function_(callback_function), message_buffer_(message_buffer) {}

    ~SubscriberCallback() = default;

    void invoke() override
    {
      try
      {
        if (!message_buffer_->empty())
        {
          callback_function_(message_buffer_->front());
          message_buffer_->erase(message_buffer_->begin());
        }
        else
        {
          std::cerr << "Error: Vector is empty" << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "Exception during callback invocation: " << e.what() << std::endl;
      }
      catch (...)
      {
        std::cerr << "Unknown exception during callback invocation." << std::endl;
      }
    }

  private:
    std::function<void(std::shared_ptr<T>)> callback_function_;
    std::vector<std::shared_ptr<T>> *message_buffer_;
  };

  template <typename T>
  class SubscriberWaitSet
  {
  public:
    SubscriberWaitSet(std::function<void(std::shared_ptr<T>)> callback_function, Channel<ChannelCallback *>::SharedPtr channel)
        : callback_function_(callback_function), channel_(channel)
    {
      subscription_callback_ = std::make_unique<SubscriberCallback<T>>(callback_function_, &message_ptr_buffer_);
    }

    ~SubscriberWaitSet()
    {
      stop();
    }

    void start(eprosima::fastdds::dds::DataReader *reader)
    {
      reader_ = reader;
      stop_flag_.store(false);

      auto &status_cond = reader_->get_statuscondition();

      status_cond.set_enabled_statuses(eprosima::fastdds::dds::StatusMask::data_available());

      // Attach the StatusCondition to the WaitSet
      wait_set_.attach_condition(status_cond);

      waitset_thread_ = std::thread(&SubscriberWaitSet::run, this);
    }

    void stop()
    {
      stop_flag_.store(true);
      if (waitset_thread_.joinable())
      {
        waitset_thread_.join();
      }
    }

    int32_t get_publisher_count()
    {
      return count_.load();
    }

  private:
    void run()
    {

      while (!stop_flag_.load())
      {
        eprosima::fastdds::dds::ConditionSeq active_conditions;
        eprosima::fastrtps::Duration_t timeout{1, 0}; // Wait for 1 second

        if (wait_set_.wait(active_conditions, timeout) == ReturnCode_t::RETCODE_OK)
        {
          // printf("Data available : Time %ld\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
          for (auto condition : active_conditions)
          {

            eprosima::fastdds::dds::StatusCondition *status_cond = dynamic_cast<eprosima::fastdds::dds::StatusCondition *>(condition);
            if (nullptr != status_cond)
            {
              eprosima::fastdds::dds::Entity *entity = status_cond->get_entity();
              eprosima::fastdds::dds::StatusMask changed_statuses = entity->get_status_changes();

              if (changed_statuses.is_active(eprosima::fastdds::dds::StatusMask::data_available()))
              {

                while (reader_->take_next_sample(&data_, &info_) == ReturnCode_t::RETCODE_OK)
                {
                    if (info_.valid_data)
                    {
                        // copy
                        auto data_ptr = std::shared_ptr<T>(new T(data_), [](T* ptr) { delete ptr; });
                        message_ptr_buffer_.emplace_back(data_ptr);
                        channel_->produce(subscription_callback_.get());
                    }
                    else
                    {
                        std::cerr << "Error: Invalid data" << std::endl;
                        break;
                    }
                }
              }
            }
          }
        }
      }
    }

    std::function<void(std::shared_ptr<T>)> callback_function_;
    Channel<ChannelCallback *>::SharedPtr channel_;
    std::vector<std::shared_ptr<T>> message_ptr_buffer_;
    std::unique_ptr<SubscriberCallback<T>> subscription_callback_;
    eprosima::fastdds::dds::SampleInfo sample_info_;
    eprosima::fastdds::dds::DataReader *reader_ = nullptr;
    std::atomic<bool> stop_flag_{false};
    std::thread waitset_thread_;
    std::atomic<int32_t> count_{0};
    eprosima::fastdds::dds::WaitSet wait_set_;
    T data_;
    eprosima::fastdds::dds::SampleInfo info_;
  };

  class ISubscription
  {
  public:
    virtual ~ISubscription() = default;
    virtual int32_t get_publisher_count() = 0;
    virtual void stop() = 0;
  };

  template <typename T>
  class Subscription : public ISubscription, public std::enable_shared_from_this<Subscription<T>>
  {
  public:
    Subscription(eprosima::fastdds::dds::DomainParticipant *participant, const std::string &topic,
                 const QoS &qos, std::function<void(std::shared_ptr<T>)> callback_function,
                 Channel<ChannelCallback *>::SharedPtr channel)
        : participant_(participant), waitset_(callback_function, channel), topic_(nullptr), subscriber_(nullptr), reader_(nullptr)
    {
      using ParentType = typename ParentTypeTraits<T>::Type;
      message_type_ = lwrcl::MessageType(new ParentType());
      lwrcl::dds::TopicQos topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT;
      if (message_type_.get_type_support().register_type(participant_) != ReturnCode_t::RETCODE_OK)
      {
        throw std::runtime_error("Failed to register message type");
      }

      eprosima::fastdds::dds::Topic *retrieved_topic = dynamic_cast<eprosima::fastdds::dds::Topic *>(participant->lookup_topicdescription(topic));
      if (retrieved_topic == nullptr)
      {
        topic_ = participant_->create_topic(topic, message_type_.get_type_support().get_type_name(), topic_qos);
        if (!topic_)
        {
          throw std::runtime_error("Failed to create topic");
        }
        topic_owned_ = true;
      }
      else
      {
        topic_ = retrieved_topic;
        topic_owned_ = false;
      }

      subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
      if (!subscriber_)
      {
        participant_->delete_topic(topic_);
        throw std::runtime_error("Failed to create subscriber");
      }
      eprosima::fastdds::dds::DataReaderQos reader_qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
      reader_qos.endpoint().history_memory_policy = eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
      reader_qos.history().depth = qos.get_depth();
      if(qos.get_history() == QoS::HistoryPolicy::KEEP_ALL)
      {
        reader_qos.history().kind = eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
      }
      else
      {
        reader_qos.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
      }
      if(qos.get_reliability() == QoS::ReliabilityPolicy::BEST_EFFORT)
      {
        reader_qos.reliability().kind = eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
      }
      else
      {
        reader_qos.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
      }
      if(qos.get_durability() == QoS::DurabilityPolicy::VOLATILE)
      {
        reader_qos.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
      }
      else
      {
        reader_qos.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
      }
      reader_qos.data_sharing().automatic();
      reader_qos.properties().properties().emplace_back("fastdds.intraprocess_delivery", "true");

      reader_ = subscriber_->create_datareader(topic_, reader_qos, nullptr);
      if (!reader_)
      {
        participant_->delete_subscriber(subscriber_);
        participant_->delete_topic(topic_);
        throw std::runtime_error("Failed to create datareader");
      }

      waitset_.start(reader_);
    }

    ~Subscription()
    {
      waitset_.stop();
      if (reader_ != nullptr)
      {
        subscriber_->delete_datareader(reader_);
      }
      if (subscriber_ != nullptr)
      {
        participant_->delete_subscriber(subscriber_);
      }
      if (topic_ != nullptr && topic_owned_)
      {
        participant_->delete_topic(topic_);
      }
    }

    int32_t get_publisher_count()
    {
      return waitset_.get_publisher_count();
    }
    using SharedPtr = std::shared_ptr<Subscription<T>>;

    void stop()
    {
      waitset_.stop();
    }

  private:
    eprosima::fastdds::dds::DomainParticipant *participant_;
    SubscriberWaitSet<T> waitset_;
    eprosima::fastdds::dds::Topic *topic_;
    eprosima::fastdds::dds::Subscriber *subscriber_;
    eprosima::fastdds::dds::DataReader *reader_;
    lwrcl::MessageType message_type_;
    bool topic_owned_;
  };

} // namespace lwrcl

#endif // LWRCL_SUBSCRIBER_HPP_
