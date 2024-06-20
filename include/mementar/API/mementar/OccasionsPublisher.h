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

    void insert(const Fact& fact, time_t stamp = time(nullptr));

  private:
    compat::onto_ros::Publisher<compat::StampedString> pub_;

    void publish(const std::string& str, time_t stamp = time(nullptr));
  };

} // namespace mementar

#endif // MEMENTAR_API_OCCASIONSPUBLISHER_H
