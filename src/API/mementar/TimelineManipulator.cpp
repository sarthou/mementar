#include "include/mementar/API/mementar/TimelineManipulator.h"

namespace mementar {

    TimelineManipulator::TimelineManipulator(const std::string& name)
            : fact_feeder_(name),
              action_feeder_(name),
              actions_(name),
              facts_(name),
              manager_(name),
              name_(name),
              inst_manager_(name)
    {}

    bool TimelineManipulator::waitInit(int32_t timeout) {
        // todo:
        //std::string servive_name = (name_ == "") ? "mementar/manage_instance" : "mementar/manage_instance/" + name_;
        //return ros::service::waitForService(servive_name, timeout);
    }

} // namespace mementar
