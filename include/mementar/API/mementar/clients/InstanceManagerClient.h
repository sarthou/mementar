#ifndef MEMENTAR_INSTANCEMANAGERCLIENT_H
#define MEMENTAR_INSTANCEMANAGERCLIENT_H

#include "ClientBase.h"

namespace mementar {
  class InstanceManagerClient : public ClientBase
  {
  public:
    explicit InstanceManagerClient(const std::string& name) : ClientBase(name.empty() ? "manage_instance" : "manage_instance/" + name) {}

    bool save(const std::string& name);
    bool draw(const std::string& name);

  private:
    std::string name_;
  };
} // namespace mementar

#endif // MEMENTAR_INSTANCEMANAGERCLIENT_H
