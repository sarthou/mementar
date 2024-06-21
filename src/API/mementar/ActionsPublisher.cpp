#include "include/mementar/API/mementar/ActionsPublisher.h"

#include <ctime>
#include <string>

#include "mementar/compat/ros.h"

namespace mementar {

  ActionsPublisher::ActionsPublisher(const std::string& name) : pub_(name.empty() ? "mementar/insert_action" : "mementar/insert_action/" + name, 1000) {}

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

} // namespace mementar
