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
    void insert(const std::string& name, compat::mem_ros::Time start_stamp = compat::mem_ros::Node::get().current_time(), compat::mem_ros::Time end_stamp = compat::mem_ros::Time(0));

    void insertEnd(const std::string& name, time_t end_stamp = time(nullptr));
    void insertEnd(const std::string& name, compat::mem_ros::Time end_stamp = compat::mem_ros::Node::get().current_time());

  private:
    compat::mem_ros::Publisher<compat::MementarAction> pub_;

    void publish(const std::string& name, time_t start_stamp, time_t end_stamp);
    void publish(const std::string& name, compat::mem_ros::Time start_stamp, compat::mem_ros::Time end_stamp);
  };

} // namespace mementar

#endif // MEMENTAR_API_ACTIONSPUBLISHER_H
