#include "include/mementar/API/mementar/ActionsPublisher.h"

#include <ctime>
#include <string>

#include "mementar/compat/ros.h"

namespace mementar {

  ActionsPublisher::ActionsPublisher(const std::string& name) : feeder_notification_callback_([](auto& msg) { (void)msg; }),
                                                                pub_(name.empty() ? "mementar/insert_action" : "mementar/insert_action/" + name, 1000),
                                                                feeder_notif_sub_(name.empty() ? "mementar/feeder_notifications" : "mementar/feeder_notifications/" + name, 1000, &ActionsPublisher::feederNotificationCallback, this)
  {}

  void ActionsPublisher::insert(const std::string& name, time_t start_stamp, time_t end_stamp)
  {
    publish(name, start_stamp, end_stamp);
  }

  void ActionsPublisher::insert(const std::string& name, compat::mem_ros::Time start_stamp, compat::mem_ros::Time end_stamp)
  {
    publish(name, start_stamp, end_stamp);
  }

  void ActionsPublisher::insertEnd(const std::string& name, time_t end_stamp)
  {
    publish(name, 0, end_stamp);
  }

  void ActionsPublisher::insertEnd(const std::string& name, compat::mem_ros::Time end_stamp)
  {
    publish(name, compat::mem_ros::Time(0), end_stamp);
  }

  void ActionsPublisher::publish(const std::string& name, time_t start_stamp, time_t end_stamp)
  {
    compat::MementarAction msg;
    msg.name = name;
    msg.start_stamp.seconds = start_stamp;
    msg.end_stamp.seconds = end_stamp;
    pub_.publish(msg);
  }

  void ActionsPublisher::publish(const std::string& name, compat::mem_ros::Time start_stamp, compat::mem_ros::Time end_stamp)
  {
    compat::MementarAction msg;
    msg.name = name;
    msg.start_stamp.seconds = start_stamp.seconds();
    msg.start_stamp.nanoseconds = start_stamp.nanoseconds();
    msg.end_stamp.seconds = end_stamp.seconds();
    msg.end_stamp.nanoseconds = end_stamp.nanoseconds();
    pub_.publish(msg);
  }

  void ActionsPublisher::feederNotificationCallback(const compat::mem_ros::MessageWrapper<std_msgs_compat::String>& msg)
  {
    feeder_notification_callback_(msg->data);
  }

} // namespace mementar
