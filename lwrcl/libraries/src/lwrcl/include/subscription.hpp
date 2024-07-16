#ifndef LWRCL_SUBSCRIBER_HPP_
#define LWRCL_SUBSCRIBER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>

#include "fast_dds_header.hpp"
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
  class SubscriberListener : public dds::DataReaderListener
  {
  public:
    virtual ~SubscriberListener()
    {
    }

    void on_subscription_matched(dds::DataReader *, const dds::SubscriptionMatchedStatus &status) override
    {
      count = status.current_count;
    }

    void on_data_available(dds::DataReader *reader) override
    {
      T temp_instance;
      if (reader->take_next_sample(&temp_instance, &sample_info_) == ReturnCode_t::RETCODE_OK && sample_info_.valid_data)
      {
        auto data_ptr = std::make_shared<T>(temp_instance);
        message_ptr_buffer_.emplace_back(data_ptr);
        channel_->produce(subscription_callback_.get());
      }
    }

    SubscriberListener(MessageType::SharedPtr message_type, std::function<void(std::shared_ptr<T>)> callback_function, Channel<ChannelCallback*>::SharedPtr channel)
        : message_type_(message_type), callback_function_(callback_function), channel_(channel)
    {
      subscription_callback_ = std::make_unique<SubscriberCallback<T>>(callback_function_, &message_ptr_buffer_);
    }
    std::atomic<int32_t> count{0};

  private:
    MessageType::SharedPtr message_type_;
    std::function<void(std::shared_ptr<T>)> callback_function_;
    Channel<ChannelCallback*>::SharedPtr channel_;
    std::vector<std::shared_ptr<T>> message_ptr_buffer_;
    std::unique_ptr<SubscriberCallback<T>> subscription_callback_;
    dds::SampleInfo sample_info_;
  };

  class ISubscription
  {
  public:
    virtual ~ISubscription() = default;
    virtual int32_t get_publisher_count() = 0;
  };

  template <typename T>
  class Subscription : public ISubscription, public std::enable_shared_from_this<Subscription<T>>
  {
  public:
    Subscription(dds::DomainParticipant *participant, MessageType::SharedPtr message_type, const std::string &topic,
               const uint16_t &depth, std::function<void(std::shared_ptr<T>)> callback_function,
               Channel<ChannelCallback*>::SharedPtr channel)
        : participant_(participant),
          listener_(message_type, callback_function, channel)
    {
      lwrcl::dds::TopicQos qos = lwrcl::dds::TOPIC_QOS_DEFAULT;
      if (message_type->get_type_support().register_type(participant_) != ReturnCode_t::RETCODE_OK)
      {
        throw std::runtime_error("Failed to register message type");
      }
      
      dds::Topic* retrieved_topic = dynamic_cast<eprosima::fastdds::dds::Topic*>(participant->lookup_topicdescription(topic));
      if (retrieved_topic == nullptr)
      {
        topic_ = participant_->create_topic(topic, message_type->get_type_support().get_type_name(), qos);
        if (!topic_)
        {
          throw std::runtime_error("Failed to create topic");
        }
      }
      else
      {
        topic_ = retrieved_topic;
      }

      subscriber_ = participant_->create_subscriber(dds::SUBSCRIBER_QOS_DEFAULT);
      if (!subscriber_)
      {
        participant_->delete_topic(topic_);
        throw std::runtime_error("Failed to create subscriber");
      }
      dds::DataReaderQos reader_qos = dds::DATAREADER_QOS_DEFAULT;
      reader_qos.endpoint().history_memory_policy = rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
      reader_qos.history().depth = depth;
      reader_qos.reliability().kind = dds::RELIABLE_RELIABILITY_QOS;
      // reader_qos.durability().kind = dds::TRANSIENT_LOCAL_DURABILITY_QOS;
      reader_qos.data_sharing().automatic();
      reader_ = subscriber_->create_datareader(topic_, reader_qos, &listener_);
      if (!reader_)
      {
        participant_->delete_subscriber(subscriber_);
        participant_->delete_topic(topic_);
        throw std::runtime_error("Failed to create datareader");
      }
    }

    ~Subscription()
    {
      if (reader_ != nullptr)
      {
        subscriber_->delete_datareader(reader_);
      }
      if (subscriber_ != nullptr)
      {
        participant_->delete_subscriber(subscriber_);
      }
      if (topic_ != nullptr)
      {
        participant_->delete_topic(topic_);
      }
    }

    int32_t get_publisher_count()
    {
      return listener_.count.load();
    }

  using SharedPtr = std::shared_ptr<Subscription<T>>;

  private:
    dds::DomainParticipant *participant_;
    SubscriberListener<T> listener_;
    dds::Topic *topic_;
    dds::Subscriber *subscriber_;
    dds::DataReader *reader_;
  };

} // namespace lwrcl

#endif // LWRCL_SUBSCRIBER_HPP_