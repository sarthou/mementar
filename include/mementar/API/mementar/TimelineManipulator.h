#ifndef MEMENTAR_TIMELINEMANIPULATOR_H
#define MEMENTAR_TIMELINEMANIPULATOR_H

#include <string>

#include "mementar/compat/ros.h"
#include "OccasionsPublisher.h"
#include "ActionsPublisher.h"
#include "mementar/API/mementar/clients/ActionClient.h"
#include "mementar/API/mementar/clients/FactClient.h"
#include "mementar/API/mementar/clients/ManagerClient.h"
#include "mementar/API/mementar/clients/InstanceManagerClient.h"

namespace mementar {
    class TimelineManipulator {
    public:
        TimelineManipulator(const std::string &name = "");

        bool waitInit(int32_t timeout = -1);

        /*size_t nb() {return actions_.nb();}
        void resetNb() {actions_.resetNb();}*/

        void verbose(bool verbose) { ClientBase::verbose(verbose); }

        OccasionsPublisher fact_feeder_;
        ActionsPublisher action_feeder_;
        ActionClient actions_;
        FactClient facts_;
        ManagerClient manager_;
        InstanceManagerClient inst_manager_;
    private:
        std::string name_;
    };

} // namespace mementar

#endif // MEMENTAR_TIMELINEMANIPULATOR_H
