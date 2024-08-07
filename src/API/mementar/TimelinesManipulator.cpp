#include "include/mementar/API/mementar/TimelinesManipulator.h"

#include <cstdint>
#include <memory>
#include <string>

#include "include/mementar/API/mementar/TimelineManipulator.h"
#include "include/mementar/API/mementar/clients/ManagerClient.h"

namespace mementar {

  bool TimelinesManipulator::waitInit(int32_t timeout)
  {
    return client_.wait(timeout);
  }

  TimelineManipulator* TimelinesManipulator::operator[](const std::string& name)
  {
    if(manipulators_.find(name) != manipulators_.end())
    {
      return manipulators_[name].get();
    }
    else
    {
      return nullptr;
    }
  }

  TimelineManipulator* TimelinesManipulator::get(const std::string& name)
  {
    if(manipulators_.find(name) != manipulators_.end())
    {
      return manipulators_[name].get();
    }
    else
    {
      return nullptr;
    }
  }

  bool TimelinesManipulator::add(const std::string& name)
  {
    if(manipulators_.find(name) != manipulators_.end())
    {
      return true;
    }
    else
    {
      if(ManagerClient::add(name) == false)
      {
        return false;
      }
      else
      {
        manipulators_[name] = std::make_unique<TimelineManipulator>(name);
        return true;
      }
    }
  }

  bool TimelinesManipulator::del(const std::string& name)
  {
    if(manipulators_.find(name) == manipulators_.end())
      return true;
    else
    {
      if(ManagerClient::del(name) == false)
        return false;
      else
      {
        manipulators_.erase(name);
        return true;
      }
    }
  }

} // namespace mementar
