#include "include/mementar/API/mementar/TimelineManipulator.h"

#include <cstdint>
#include <string>

namespace mementar {

  TimelineManipulator::TimelineManipulator(const std::string& name)
    : fact_feeder(name),
      action_feeder(name),
      actions(name),
      facts(name),
      inst_manager(name),
      name_(name)
  {}

  bool TimelineManipulator::waitInit(int32_t timeout)
  {
    return inst_manager.client_.wait(timeout);
  }

} // namespace mementar
