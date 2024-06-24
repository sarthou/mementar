#ifndef MEMENTAR_API_OCCASIONSSUBSCRIBER_H
#define MEMENTAR_API_OCCASIONSSUBSCRIBER_H

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "Fact.h"
#include "mementar/compat/ros.h"

namespace mementar {

  class OccasionsSubscriber
  {
  public:
    OccasionsSubscriber(const std::function<void(const Fact&)>& callback, const std::string& name = "", bool spin_thread = true);
    OccasionsSubscriber(const std::function<void(const Fact&)>& callback, bool spin_thread);
    ~OccasionsSubscriber();

    bool subscribe(const Fact& pattern, size_t count = -1);
    bool cancel();

    bool end() const { return ids_.empty(); }

  private:
    compat::mem_ros::Subscriber<compat::MementarOccasion> sub_;
    compat::mem_ros::Client<compat::MementarOccasionSubscription> client_subscribe_;
    compat::mem_ros::Client<compat::MementarOccasionUnsubscription> client_cancel_;

    std::atomic<bool> need_to_terminate_;

    std::vector<size_t> ids_;

    void occasionCallback(const compat::MementarOccasion& msg);

    std::function<void(const Fact&)> callback_;

    void spinThread();
  };

} // namespace mementar

#endif // MEMENTAR_API_OCCASIONSSUBSCRIBER_H
