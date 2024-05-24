#ifndef MEMENTAR_FEEDERECHO_H
#define MEMENTAR_FEEDERECHO_H

#include <mutex>

#include "mementar/compat/ros.h"
#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"

namespace mementar {

class FeederEcho
{
public:
  explicit FeederEcho(const std::string& echo_topic) : feeder_echo_pub_(echo_topic, 1000)
  {}

  ~FeederEcho()
  {
    mut_.lock();
    echo_messages.clear();
  }

  void add(ContextualizedFact* fact)
  {
    mut_.lock();
    echo_messages.emplace_back(fact);
    mut_.unlock();
  }

  void publish()
  {
    mut_.lock();
    compat::StampedFact ros_msg;
    for(const auto& message : echo_messages)
    {
      const auto rosTime = compat::onto_ros::Time(message->getTime());

      ros_msg.id = message->getId();
      ros_msg.stamp.seconds = rosTime.seconds();
      ros_msg.stamp.nanoseconds = rosTime.nanoseconds();
      ros_msg.subject = message->subject_;
      ros_msg.predicat = message->predicat_;
      ros_msg.object = message->object_;
      ros_msg.added = message->add_;
      feeder_echo_pub_.publish(ros_msg);
    }
    echo_messages.clear();
    mut_.unlock();
  }

private:
  std::mutex mut_;
  compat::onto_ros::Publisher<compat::StampedFact> feeder_echo_pub_;
  std::vector<ContextualizedFact*> echo_messages;
};

} // namespace mementar

#endif // MEMENTAR_FEEDERECHO_H
