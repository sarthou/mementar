#include "mementar/API/mementar/clients/InstanceManagerClient.h"

#include <cstdint>
#include <string>

namespace mementar {
  int16_t InstanceManagerClient::save(const std::string& name)
  {
    return callCode("save", name);
  }

  int16_t InstanceManagerClient::draw(const std::string& name)
  {
    return callCode("draw", name);
  }
} // namespace mementar
