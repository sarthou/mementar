#include "mementar/API/mementar/OccasionsPublisher.h"

#include <ctime>
#include <string>

#include "mementar/API/mementar/Fact.h"

namespace mementar {

  OccasionsPublisher::OccasionsPublisher(const std::string& name)
    : pub_((name.empty()) ? "mementar/insert_fact_stamped" : "mementar/insert_fact_stamped/" + name, 1000) {}

  void OccasionsPublisher::insert(const Fact& fact, time_t stamp)
  {
    publish(fact(), stamp);
  }

  void OccasionsPublisher::publish(const std::string& str, time_t stamp)
  {
    compat::StampedString msg;
    msg.data = str;
    msg.stamp.seconds = stamp;
    pub_.publish(msg);
  }

} // namespace mementar
