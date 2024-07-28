# LWRCL (Lightweight rclcpp Compatible Library) for Fast DDS

This library provides a simplified API similar to ROS 2's rclcpp for working with Fast DDS, enabling easier integration and management of nodes, publishers, subscribers, and timers within the Fast DDS ecosystem.

## Features

- Simplified node creation and management within a Fast DDS domain.
- Efficient message publishing and subscription handling with callback support.
- Seamless communication with ROS 2 topics without needing special modifications.
- Compatible with lightweight SBCs such as Raspberry Pi for easy integration with ROS 2 ecosystems.
- Periodic task execution using timers.
- Supports custom message types for flexible communication needs.
- Executors for managing and spinning multiple nodes concurrently.

## API Overview
  
### Node Management

- **get_name**: Retrieves the name of the node.
- **get_clock**: Returns the clock associated with the node.
- **create_publisher**: Establishes a new message publisher on a specified topic.
- **create_subscription**: Creates a subscription for receiving messages on a specified topic with a callback function.
- **create_timer**: Sets up a system clock timer to call a function at a specified interval.
- **create_wall_timer**: Sets up a monotonic timer to call a function at a specified interval.
- **make_shared**: Creates a shared pointer to a new node.
- **spin**: Continuously processes incoming messages and executes callbacks.
- **spin_some**: Processes available messages without blocking.
- **stop_spin**: Stops the continuous message processing loop.
- **shutdown**: Stops the continuous message processing loop.
- **set_parameter**: Sets a parameter value for the specified node.
- **get_parameter**: Retrieves the value of a parameter for the specified node.
- **declare_parameter**: Declares a parameter for the specified node.

### Publisher

- **create_publisher**: Establishes a new message publisher on a specified topic.
- **publish**: Sends messages to the associated topic.
- **get_subscriber_count**: Retrieves the number of subscribers currently connected to the publisher.

### Subscription

- **create_subscription**: Creates a subscription for receiving messages on a specified topic with a callback function.
- **get_publisher_count**: Counts the number of publishers to which the subscriber is connected.

### Timer

- **create_timer**: Sets up a system clock timer to call a function at a specified interval.
- **create_wall_timer**: Sets up a monotonic timer to call a function at a specified interval.
- **stop_timer**: Halts the timer.

## Time, Duration, Clock, and Rate Implementation

This section outlines the implementation details of the Time, Duration, Clock, and Rate classes, which are essential for handling timing and scheduling within the system.

### Time Implementation

The `Time` class provides functionality to represent a point in time or a duration of time with nanosecond precision.

- **Constructors:**
  - Default constructor initializes time to 0 nanoseconds.
  - Constructor accepting an `int64_t` for nanoseconds.
  - Constructor accepting an `int32_t` for seconds and a `uint32_t` for additional nanoseconds.

- **Methods:**
  - `nanoseconds()`: Returns the time as an integer number of nanoseconds.
  - `seconds()`: Returns the time as a floating-point number of seconds.

### Duration Implementation

The `Duration` class represents a time span with nanosecond precision.

- **Constructors:**
  - Default constructor initializes duration to 0 nanoseconds.
  - Constructor accepting an `std::chrono` data type.
  - Constructor accepting an `int64_t` for nanoseconds.
  - Constructor accepting an `int32_t` for seconds and a `uint32_t` for additional nanoseconds.

- **Methods:**
  - `nanoseconds()`: Returns the duration as an integer number of nanoseconds.
  - `seconds()`: Returns the duration as a floating-point number of seconds.

### Clock Implementation

The `Clock` class is used to access the current time, based on the clock type.

- **Constructor:** Accepts a `ClockType` enumeration to specify the clock type (e.g., `SYSTEM_TIME`).
- **Methods:**
  - `now()`: Returns the current time as a `Time` object, based on the clock's type.
  - `get_clock_type()`: Returns the type of the clock.

### Rate Implementation

The `Rate` class is designed to maintain a specified rate of loop iteration, using sleep to delay execution.

- **Constructor:** Accepts a `Duration` object that specifies the desired period between iterations.
- **Method:**
  - `sleep()`: Sleeps until the next iteration should start, adjusting for any drift to maintain the rate.

These classes provide a foundational framework for precise time management and rate control in the system, allowing for time-based scheduling and execution control.

### Parameter Server

The `ParameterServer` class provides a centralized storage mechanism for managing parameters within the Fast DDS ecosystem. It allows nodes to set, get, and declare parameters, enabling efficient parameter management across multiple nodes.

### Parameter Client

The `ParameterClient` class enables nodes to access and interact with parameters stored in the `ParameterServer`. It provides methods for retrieving parameter values, checking parameter existence, and updating parameter values.

### QoS Profiles

User can define QoS depth for publishers and subscribers. The QoS depth determines the maximum number of messages that can be stored in the queue before messages are dropped.

### Logging

The `Logger` class provides a simple logging mechanism for printing messages to the console. It supports different log levels, including `DEBUG`, `INFO`, `WARN`, and `ERROR`, allowing users to control the verbosity of log messages.


# Executors for Fast DDS

Executors play a crucial role in the RCL-like wrapper for Fast DDS, allowing for concurrent message processing and event handling across multiple nodes. Inspired by the ROS 2 executor concept, these executors facilitate the management and operation of nodes, enabling efficient communication within the Fast DDS ecosystem.

## SingleThreadedExecutor

The `SingleThreadedExecutor` manages and spins multiple nodes sequentially within a single thread. This executor is designed for simplicity and is best suited for scenarios where tasks need to be executed in a specific order without the overhead of multi-threading.

### Features

- **Sequential Processing:** Executes node callbacks one at a time, ensuring that message processing does not overlap.
- **Simplicity:** Easier to debug and maintain due to the single-threaded nature of operation.
- **Use Case:** Ideal for simpler systems where concurrent message processing is not critical, or for tasks that must be executed in order.

### Key Functions

- **add_node(std::shared_ptr\<Node\> node):** Integrates a node into the executor's workflow.
- **remove_node(std::shared_ptr\<Node\> node):** Detaches a node from the executor.
- **spin():** Begins the sequential processing of messages for all nodes managed by the executor.
- **cancel():** Halts the processing loop, ensuring all nodes are gracefully stopped.

## Executors for Fast DDS

Executors are a key component of the Fast DDS ecosystem, enabling the concurrent processing of messages and events across multiple nodes. By managing the execution of nodes within a single or multiple threads, executors provide a flexible and efficient way to handle communication and data processing tasks.

### Features

- **Concurrent Processing:** Allows nodes to run in parallel, optimizing performance and responsiveness.
- **Scalability:** Efficiently manages a larger number of nodes, making it suitable for complex systems.
- **Flexibility:** Offers the ability to handle variable loads and tasks that are independent of each other.
- **Use Case:** Best for applications requiring real-time processing or when multiple nodes need to operate independently without blocking each other.

### MultiThreadedExecutor

The `MultiThreadedExecutor` extends the functionality of the SingleThreadedExecutor by allowing nodes to be spun in parallel across multiple threads. This executor is capable of handling more complex systems where tasks need to run concurrently, optimizing performance and responsiveness.

### SingleThreadedExecutor

The `SingleThreadedExecutor` manages and spins multiple nodes sequentially within a single thread. This executor is designed for simplicity and is best suited for scenarios where tasks need to be executed in a specific order without the overhead of multi-threading.

### Key Functions

- **add_node(std::shared_ptr\<Node\> node):** Adds a node to be managed concurrently by the executor.
- **remove_node(std::shared_ptr\<Node\> node):** Removes a node from the concurrent processing pool.
- **spin():** Starts concurrent message processing for all nodes, leveraging multi-threading to achieve parallel execution.
- **cancel():** Stops all threads and ensures a clean shutdown of node operations.

## Compatibility with ROS 2

User can use `rclcpp` namespace for using `lwrcl` library. This allows for seamless integration with ROS 2 topics and messages, enabling communication between nodes in the Fast DDS ecosystem and the ROS 2 environment.

### ROS 2 Compatible Data Types

This repository contains a set of ROS 2 compatible data types that can be used with the `lwrcl` library. These data types are designed to be compatible with ROS 2 messages, allowing for easy communication between nodes in the Fast DDS ecosystem and the ROS 2 environment.

### ROS 2 Compatible Custom Messages

User can define custom messages that are compatible with ROS 2 message types. These custom messages can be used with the `lwrcl` library to facilitate communication between nodes in the Fast DDS ecosystem and the ROS 2 environment.



## License

This project is a fork and has been modified under the terms of the Apache 2.0 license. The original work is also licensed under Apache 2.0. See the LICENSE file for more details.


