#ifndef MEMENTAR_MANAGERCLIENT_H
#define MEMENTAR_MANAGERCLIENT_H

#include "ClientBase.h"

namespace mementar {
  class ManagerClient : public ClientBase
  {
  public:
    ManagerClient() : ClientBase("manage") {}

    std::vector<std::string> list();

    bool add(const std::string& name);
    bool copy(const std::string& name);
    bool del(const std::string& name); // maybe name this to something like "remove"??

  private:
  };
} // namespace mementar

#endif // MEMENTAR_MANAGERCLIENT_H
