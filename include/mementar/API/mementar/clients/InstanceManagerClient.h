#ifndef MEMENTAR_INSTANCEMANAGERCLIENT_H
#define MEMENTAR_INSTANCEMANAGERCLIENT_H

#include "ClientBase.h"

namespace mementar {
  class InstanceManagerClient : public ClientBase
  {
  public:
    explicit InstanceManagerClient(const std::string& name) : ClientBase(name.empty() ? "manage_instance" : "manage_instance/" + name) {}

    int16_t save(const std::string& name);
    int16_t draw(const std::string& name);

  private:
    std::string name_;
  };
} // namespace mementar

#endif // MEMENTAR_INSTANCEMANAGERCLIENT_H
