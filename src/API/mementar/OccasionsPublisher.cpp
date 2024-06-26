#include "mementar/API/mementar/OccasionsPublisher.h"

#include <ctime>
#include <string>

#include "mementar/API/mementar/Fact.h"
#include "mementar/compat/ros.h"

namespace mementar {

  OccasionsPublisher::OccasionsPublisher(const std::string& name) : feeder_notification_callback_([](auto& msg) { (void)msg; }),
                                                                    pub_((name.empty()) ? "mementar/insert_fact_stamped" : "mementar/insert_fact_stamped/" + name, 1000),
                                                                    feeder_notif_sub_(name.empty() ? "mementar/feeder_notifications" : "mementar/feeder_notifications/" + name, 1000, &OccasionsPublisher::feederNotificationCallback, this)
  {}

  void OccasionsPublisher::insert(const Fact& fact, const compat::mem_ros::Time& stamp)
  {
    publish(fact(), stamp);
  }

  void OccasionsPublisher::insert(const Fact& fact, time_t stamp)
  {
    publish(fact(), stamp);
  }

  void OccasionsPublisher::publish(const std::string& str, const compat::mem_ros::Time& stamp)
  {
    compat::StampedString msg;
    msg.data = str;
    msg.stamp.seconds = stamp.seconds();
    msg.stamp.nanoseconds = stamp.nanoseconds();
    pub_.publish(msg);
  }

  void OccasionsPublisher::publish(const std::string& str, time_t stamp)
  {
    compat::StampedString msg;
    msg.data = str;
    msg.stamp.seconds = stamp;
    msg.stamp.nanoseconds = 0;
    pub_.publish(msg);
  }

  void OccasionsPublisher::feederNotificationCallback(const compat::mem_ros::MessageWrapper<std_msgs_compat::String>& msg)
  {
    feeder_notification_callback_(msg->data);
  }

} // namespace mementar
