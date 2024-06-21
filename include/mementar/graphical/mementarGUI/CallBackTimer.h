#ifndef MEMENTAR_CALLBACKTIMER_H
#define MEMENTAR_CALLBACKTIMER_H

#include <atomic>
#include <functional>
#include <thread>

class CallBackTimer
{
public:
  CallBackTimer() : execute_(false)
  {}

  ~CallBackTimer()
  {
    if(execute_.load(std::memory_order_acquire))
      stop();
  }

  void stop()
  {
    execute_.store(false, std::memory_order_release);
    if(th_.joinable())
      th_.join();
  }

  void start(int interval, const std::function<void(void)>& func)
  {
    if(execute_.load(std::memory_order_acquire))
      stop();

    execute_.store(true, std::memory_order_release);
    th_ = std::thread([this, interval, func]() {
      while(execute_.load(std::memory_order_acquire))
      {
        func();
        std::this_thread::sleep_for(
          std::chrono::milliseconds(interval));
      }
    });
  }

  bool isRunning() const noexcept
  {
    return (execute_.load(std::memory_order_acquire) &&
            th_.joinable());
  }

private:
  std::atomic<bool> execute_;
  std::thread th_;
};

#endif // MEMENTAR_CALLBACKTIMER_H
