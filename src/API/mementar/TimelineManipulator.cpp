#include "include/mementar/API/mementar/TimelineManipulator.h"

#include <cstdint>
#include <string>

namespace mementar {

  TimelineManipulator::TimelineManipulator(const std::string& name)
    : fact_feeder_(name),
      action_feeder_(name),
      actions_(name),
      facts_(name),
      manager_(name),
      inst_manager_(name),
      name_(name)
  {}

  bool TimelineManipulator::waitInit(int32_t timeout)
  {
    return inst_manager_.client_.wait(timeout);
  }

} // namespace mementar
