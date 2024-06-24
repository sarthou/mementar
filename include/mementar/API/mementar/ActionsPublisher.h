#ifndef MEMENTAR_API_ACTIONSPUBLISHER_H
#define MEMENTAR_API_ACTIONSPUBLISHER_H

#include <string>

#include "mementar/compat/ros.h"

namespace mementar {

  class ActionsPublisher
  {
  public:
    explicit ActionsPublisher(const std::string& name = "");

    void insert(const std::string& name, time_t start_stamp = time(nullptr), time_t end_stamp = 0);
    void insert(const std::string& name, compat::mem_ros::Time start_stamp = compat::mem_ros::Node::get().currentTime(), compat::mem_ros::Time end_stamp = compat::mem_ros::Time(0));

    void insertEnd(const std::string& name, time_t end_stamp = time(nullptr));
    void insertEnd(const std::string& name, compat::mem_ros::Time end_stamp = compat::mem_ros::Node::get().currentTime());

    /// @brief Register a callback function to get notifications from the feeder.
    /// @param callback is the callback function taking a string.
    void registerFeederNotificationCallback(const std::function<void(const std::string&)>& callback) { feeder_notification_callback_ = callback; }

  private:
    std::function<void(const std::string&)> feeder_notification_callback_;

    compat::mem_ros::Publisher<compat::MementarAction> pub_;
    compat::mem_ros::Subscriber<std_msgs_compat::String> feeder_notif_sub_;

    void publish(const std::string& name, time_t start_stamp, time_t end_stamp);
    void publish(const std::string& name, compat::mem_ros::Time start_stamp, compat::mem_ros::Time end_stamp);

    void feederNotificationCallback(const compat::mem_ros::MessageWrapper<std_msgs_compat::String>& msg);
  };

} // namespace mementar

#endif // MEMENTAR_API_ACTIONSPUBLISHER_H
