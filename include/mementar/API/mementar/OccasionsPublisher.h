#ifndef MEMENTAR_API_OCCASIONSPUBLISHER_H
#define MEMENTAR_API_OCCASIONSPUBLISHER_H

#include <ctime>
#include <string>

#include "mementar/API/mementar/Fact.h"
#include "mementar/compat/ros.h"

namespace mementar {

  class OccasionsPublisher
  {
  public:
    OccasionsPublisher(const std::string& name = "");

    void insert(const Fact& fact, const mementar::compat::mem_ros::Time& stamp = compat::mem_ros::Node::get().currentTime());
    void insert(const Fact& fact, time_t end_stamp);

    /// @brief Register a callback function to get notifications from the feeder.
    /// @param callback is the callback function taking a string.
    void registerFeederNotificationCallback(const std::function<void(const std::string&)>& callback) { feeder_notification_callback_ = callback; }

  private:
    std::function<void(const std::string&)> feeder_notification_callback_;

    compat::mem_ros::Publisher<compat::StampedString> pub_;
    compat::mem_ros::Subscriber<std_msgs_compat::String> feeder_notif_sub_;

    void publish(const std::string& str, const mementar::compat::mem_ros::Time& stamp = compat::mem_ros::Node::get().currentTime());
    void publish(const std::string& str, time_t end_stamp);

    void feederNotificationCallback(const compat::mem_ros::MessageWrapper<std_msgs_compat::String>& msg);
  };

} // namespace mementar

#endif // MEMENTAR_API_OCCASIONSPUBLISHER_H
