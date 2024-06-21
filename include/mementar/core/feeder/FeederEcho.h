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
      echo_messages_.clear();
    }

    void add(ContextualizedFact* fact)
    {
      mut_.lock();
      echo_messages_.emplace_back(fact);
      mut_.unlock();
    }

    void publish()
    {
      mut_.lock();
      compat::StampedFact ros_msg;
      for(auto* message : echo_messages_)
      {
        const auto ros_time = compat::onto_ros::Time(message->getTime());

        ros_msg.id = message->getId();
        ros_msg.stamp.seconds = ros_time.seconds();
        ros_msg.stamp.nanoseconds = ros_time.nanoseconds();
        ros_msg.subject = message->subject_;
        ros_msg.predicat = message->predicat_;
        ros_msg.object = message->object_;
        ros_msg.added = message->add_ ? 1 : 0;
        feeder_echo_pub_.publish(ros_msg);
      }
      echo_messages_.clear();
      mut_.unlock();
    }

  private:
    std::mutex mut_;
    compat::onto_ros::Publisher<compat::StampedFact> feeder_echo_pub_;
    std::vector<ContextualizedFact*> echo_messages_;
  };

} // namespace mementar

#endif // MEMENTAR_FEEDERECHO_H
