#ifndef LWRCL_TIMER_HPP_
#define LWRCL_TIMER_HPP_

#include <atomic>
#include <functional>
#include <iostream>
#include <thread>
#include <chrono>

#include "fast_dds_header.hpp"
#include "channel.hpp"
#include "clock_time_duration.hpp"

namespace lwrcl
{

  class TimerCallback : public ChannelCallback
  {
  public:
    TimerCallback(std::function<void()> callback_function)
        : callback_function_(callback_function) {}

    ~TimerCallback() = default;

    void invoke() override
    {
      try
      {
        callback_function_();
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
    std::function<void()> callback_function_;
  };

  class ITimerBase
  {
  public:
    virtual ~ITimerBase() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
  };

  class TimerBase : public ITimerBase, public std::enable_shared_from_this<TimerBase>
  {
  public:
    TimerBase(Duration period, std::function<void()> callback_function, Channel<ChannelCallback *>::SharedPtr channel, Clock::ClockType clock_type)
        : clock_type_(clock_type), period_(period), channel_(channel), stop_flag_(false)
    {
      timer_callback_ = std::make_unique<TimerCallback>(callback_function);
      start();
    }

    ~TimerBase()
    {
      stop();
    }

    void start() override
    {
      worker_ = std::thread([this]()
                            { run(); });
    }

    void stop() override
    {
      stop_flag_ = true;
      if (worker_.joinable())
      {
        worker_.join(); // Wait for the worker thread to finish
      }
    }

    using SharedPtr = std::shared_ptr<TimerBase>;

  private:
    void run_system_time()
    {
      auto next_execution_time = std::chrono::system_clock::now() + std::chrono::nanoseconds(period_.nanoseconds());
      while (!stop_flag_)
      {
        std::this_thread::sleep_until(next_execution_time);
        channel_->produce(timer_callback_.get());
        next_execution_time += std::chrono::nanoseconds(period_.nanoseconds()); // Schedule next execution
      }
    }

    void run_steady_time()
    {
      auto next_execution_time = std::chrono::steady_clock::now() + std::chrono::nanoseconds(period_.nanoseconds());
      while (!stop_flag_)
      {
        std::this_thread::sleep_until(next_execution_time);
        channel_->produce(timer_callback_.get());
        next_execution_time += std::chrono::nanoseconds(period_.nanoseconds()); // Schedule next execution
      }
    }

    void run()
    {
      if (clock_type_ == Clock::ClockType::SYSTEM_TIME)
      {
        run_system_time();
      }
      else
      {
        run_steady_time();
      }
    }

    Clock::ClockType clock_type_;
    Duration period_;
    std::unique_ptr<TimerCallback> timer_callback_;
    std::thread worker_;
    Channel<ChannelCallback *>::SharedPtr channel_;
    bool stop_flag_;
  };

} // namespace lwrcl

#endif // LWRCL_TIMER_HPP_