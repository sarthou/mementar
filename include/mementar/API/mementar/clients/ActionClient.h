#ifndef MEMENTAR_ACTIONCLIENT_H
#define MEMENTAR_ACTIONCLIENT_H

#include "ClientBase.h"

namespace mementar {

  class ActionClient : public ClientBase
  {
  public:
    ActionClient(const std::string& name) : ClientBase(name.empty() ? "action" : "action/" + name) {}

    bool exist(const std::string& action_name);
    std::vector<std::string> getPending();
    bool isPending(const std::string& action_name);
    compat::onto_ros::Time getStartStamp(const std::string& action_name);
    compat::onto_ros::Time getEndStamp(const std::string& action_name);
    compat::onto_ros::Time getDuration(const std::string& action_name);
    std::string getStartFact(const std::string& action_name);
    std::string getEndFact(const std::string& action_name);
    std::vector<std::string> getFactsDuring(const std::string& action_name);

  private:
  };

} // namespace mementar

#endif // MEMENTAR_ACTIONCLIENT_H