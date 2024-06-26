#ifndef MEMENTAR_TIMELINESMANIPULATOR_H
#define MEMENTAR_TIMELINESMANIPULATOR_H

#include <map>
#include <string>

#include "TimelineManipulator.h"
#include "mementar/API/mementar/clients/ManagerClient.h"
#include "mementar/compat/ros.h"

namespace mementar {
  class TimelinesManipulator : public ManagerClient
  {
  public:
    TimelinesManipulator();
    ~TimelinesManipulator() = default;

    bool waitInit(int32_t timeout = -1);

    TimelineManipulator* operator[](const std::string& name);
    TimelineManipulator* get(const std::string& name);

    bool add(const std::string& name);
    bool del(const std::string& name);

    void verbose(bool verbose) { ClientBase::verbose(verbose); }

  private:
    std::map<std::string, std::unique_ptr<TimelineManipulator>> manipulators_;
  };

} // namespace mementar

#endif // MEMENTAR_TIMELINESMANIPULATOR_H
