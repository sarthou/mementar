#include "mementar/API/mementar/clients/FactClient.h"

#include <string>

#include "mementar/compat/ros.h"

namespace mementar {

  bool FactClient::exist(const std::string& fact_id)
  {
    return (callStr("exist", fact_id).empty() == false);
  }

  bool FactClient::isActionPart(const std::string& fact_id)
  {
    return (callStr("isActionPart", fact_id).empty() == false);
  }

  std::string FactClient::getActionPart(const std::string& fact_id)
  {
    return callStr("getActionPart", fact_id);
  }

  std::string FactClient::getData(const std::string& fact_id)
  {
    return callStr("getData", fact_id);
  }

  compat::onto_ros::Time FactClient::getStamp(const std::string& fact_id)
  {
    return callStamp("getStamp", fact_id);
  }

} // namespace mementar