#include "mementar/API/mementar/clients/InstanceManagerClient.h"

#include <string>

namespace mementar {

  bool InstanceManagerClient::reset()
  {
    return callBool("reset", "");
  }

  bool InstanceManagerClient::save(const std::string& name)
  {
    return callBool("save", name);
  }

  bool InstanceManagerClient::draw(const std::string& name)
  {
    return callBool("draw", name);
  }

} // namespace mementar
