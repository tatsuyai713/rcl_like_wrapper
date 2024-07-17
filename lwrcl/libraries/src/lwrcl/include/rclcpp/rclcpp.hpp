#ifndef RCLCPP_HPP_
#define RCLCPP_HPP_
#include "lwrcl.hpp"

namespace rclcpp {
    using Clock = lwrcl::Clock;
    using Node = lwrcl::Node;
    namespace executors {
        using SingleThreadedExecutor = lwrcl::executors::SingleThreadedExecutor;
        using MultiThreadedExecutor = lwrcl::executors::MultiThreadedExecutor;
    }
    using Duration = lwrcl::Duration;
    using Time = lwrcl::Time;
    using Rate = lwrcl::Rate;
    using WallRate = lwrcl::WallRate;
    template <typename MessageType>
    using Subscription = ::lwrcl::Subscription<MessageType>;
    template <typename MessageType>
    using Publisher = ::lwrcl::Publisher<MessageType>;
    
    using TimerBase = lwrcl::TimerBase;

    inline bool ok() {
        return lwrcl::ok();
    }
    inline void spin(std::shared_ptr<Node> node) {
        return lwrcl::spin(node);
    }
    inline void spin_some(std::shared_ptr<Node> node) {
        return lwrcl::spin_some(node);
    }
    inline void shutdown() {
        return lwrcl::shutdown();
    }
    inline void sleep_for(const Duration &duration) {
        return lwrcl::sleep_for(duration);
    }
    inline void init(int argc, char *argv[]) {
        return lwrcl::init(argc, argv);
    }
}

#define RCLCPP_DEBUG(logger, ...) (logger).log(lwrcl::DEBUG, __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) (logger).log(lwrcl::INFO, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) (logger).log(lwrcl::WARN, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) (logger).log(lwrcl::ERROR, __VA_ARGS__)

#endif // RCLCPP_HPP_