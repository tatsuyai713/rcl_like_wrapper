#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include "eprosima_namespace.hpp"

namespace rcl_like_wrapper {

// Class representing a message type with associated TypeSupport.
class MessageType {
public:
    // Constructor that takes a pointer to a TopicDataType and initializes the TypeSupport.
    MessageType(eprosima::fastdds::dds::TopicDataType *message_type)
        : type_support(message_type) {}

    // Copy constructor that copies the TypeSupport from another MessageType.
    MessageType(const MessageType& other)
        : type_support(other.type_support) {}

    // Default constructor that initializes the TypeSupport to nullptr.
    MessageType() : type_support(nullptr) {}

    // Copy assignment operator that assigns the TypeSupport from another MessageType.
    MessageType& operator=(const MessageType& other) {
        // Check for self-assignment to avoid unnecessary operations.
        if (this != &other) {
            type_support = other.type_support;
        }
        return *this;
    }

    // Destructor; no dynamic memory management, so default implementation is used.
    ~MessageType() {}

    // Member variable representing the TypeSupport associated with the message type.
    eprosima::fastdds::dds::TypeSupport type_support;
};

// Alias for a map that associates message type names (strings) with MessageType instances.
using MessageTypes = std::unordered_map<std::string, MessageType>;


intptr_t node_create_node(uint16_t domain_id);
void node_destroy_node(intptr_t node_ptr);
void node_spin(intptr_t node_ptr);
void node_spin_once(intptr_t node_ptr);
void node_stop_spin(intptr_t node_ptr);
intptr_t publisher_create_publisher(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos& qos);
void publisher_publish_impl(intptr_t publisher_ptr, void* message);
int32_t publisher_get_subscription_count(intptr_t publisher_ptr);
intptr_t subscription_create_subscription(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos& qos, std::function<void(void*)> callback);
int32_t subscription_get_publisher_count(intptr_t subscription_ptr);
void rcl_like_wrapper_init(const MessageTypes &types);

} // namespace rcl_like_wrapper
