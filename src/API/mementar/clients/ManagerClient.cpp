#include "mementar/API/mementar/clients/ManagerClient.h"

#include <string>
#include <vector>

namespace mementar {

  std::vector<std::string> ManagerClient::list()
  {
    return callStrs("list", "");
  }

  bool ManagerClient::add(const std::string& name)
  {
    return callBool("add", name);
  }

  bool ManagerClient::copy(const std::string& name)
  {
    return callBool("copy", name);
  }

  bool ManagerClient::del(const std::string& name)
  {
    return callBool("delete", name);
  }

} // namespace mementar
