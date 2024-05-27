#ifndef MEMENTAR_MANAGERCLIENT_H
#define MEMENTAR_MANAGERCLIENT_H

#include "ClientBase.h"

namespace mementar {
  class ManagerClient : public ClientBase
  {
  public:
    explicit ManagerClient(const std::string& name) : ClientBase(name.empty() ? "manage" : "manage/" + name) {}

    std::vector<std::string> list();

    int16_t add(const std::string& name);
    int16_t copy(const std::string& name);
    int16_t del(const std::string& name); // maybe name this to something like "remove"??

  private:
  };
} // namespace mementar

#endif // MEMENTAR_MANAGERCLIENT_H
