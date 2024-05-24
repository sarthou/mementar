#include "mementar/API/mementar/clients/InstanceManagerClient.h"

namespace mementar {
    int16_t InstanceManagerClient::save(const std::string& name) {
        return call("save", name)->code;
    }

    int16_t InstanceManagerClient::draw(const std::string& name) {
        return call("draw", name)->code;
    }
} // namespace mementar
