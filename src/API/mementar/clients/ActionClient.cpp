#include "mementar/API/mementar/clients/ActionClient.h"

#include <string>
#include <vector>

#include "mementar/compat/ros.h"

namespace mementar {

  bool ActionClient::exist(const std::string& action_name)
  {
    return (callStr("exist", action_name).empty() == false);
  }

  std::vector<std::string> ActionClient::getPending()
  {
    return callArray("getPending", "");
  }

  bool ActionClient::isPending(const std::string& action_name)
  {
    return (callStr("isPending", action_name).empty() == false);
  }

  compat::onto_ros::Time ActionClient::getStartStamp(const std::string& action_name)
  {
    return callStamp("getStartStamp", action_name);
  }

  compat::onto_ros::Time ActionClient::getEndStamp(const std::string& action_name)
  {
    return callStamp("getEndStamp", action_name);
  }

  compat::onto_ros::Time ActionClient::getDuration(const std::string& action_name)
  {
    return callStamp("getDuration", action_name);
  }

  std::string ActionClient::getStartFact(const std::string& action_name)
  {
    return callStr("getStartFact", action_name);
  }

  std::string ActionClient::getEndFact(const std::string& action_name)
  {
    return callStr("getEndFact", action_name);
  }

  std::vector<std::string> ActionClient::getFactsDuring(const std::string& action_name)
  {
    return callArray("getFactsDuring", action_name);
  }

} // namespace mementar