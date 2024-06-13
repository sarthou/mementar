#include "mementar/API/mementar/clients/ManagerClient.h"

namespace mementar {

  std::vector<std::string> ManagerClient::list()
  {
    return callArray("list", "");
  }

  int16_t ManagerClient::add(const std::string& name)
  {
    return callCode("add", name);
  }

  int16_t ManagerClient::copy(const std::string& name)
  {
    return callCode("copy", name);
  }

  int16_t ManagerClient::del(const std::string& name)
  {
    return callCode("delete", name);
  }

} // namespace mementar
