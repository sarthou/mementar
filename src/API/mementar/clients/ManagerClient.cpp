#include "mementar/API/mementar/clients/ManagerClient.h"

namespace mementar {

  std::vector<std::string> ManagerClient::list()
  {
    return callArray("list", "");
  }

  int16_t ManagerClient::add(const std::string& name)
  {
    return call("add", name)->code;
  }

  int16_t ManagerClient::copy(const std::string& name)
  {
    return call("copy", name)->code;
  }

  int16_t ManagerClient::del(const std::string& name)
  {
    return call("delete", name)->code;
  }

} // namespace mementar
