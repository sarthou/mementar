#ifndef MEMENTAR_TIMELINEMANIPULATOR_H
#define MEMENTAR_TIMELINEMANIPULATOR_H

#include <string>

#include "ActionsPublisher.h"
#include "OccasionsPublisher.h"
#include "mementar/API/mementar/clients/ActionClient.h"
#include "mementar/API/mementar/clients/FactClient.h"
#include "mementar/API/mementar/clients/InstanceManagerClient.h"
#include "mementar/API/mementar/clients/ManagerClient.h"
#include "mementar/compat/ros.h"

namespace mementar {
  class TimelineManipulator
  {
  public:
    TimelineManipulator(const std::string& name = "");

    bool waitInit(int32_t timeout = -1);

    size_t nb() { return actions.nb(); }
    void resetNb() { actions.resetNb(); }

    void verbose(bool verbose) { ClientBase::verbose(verbose); }

    OccasionsPublisher fact_feeder;
    ActionsPublisher action_feeder;
    ActionClient actions;
    FactClient facts;
    InstanceManagerClient inst_manager;

  private:
    std::string name_;
  };

} // namespace mementar

#endif // MEMENTAR_TIMELINEMANIPULATOR_H
