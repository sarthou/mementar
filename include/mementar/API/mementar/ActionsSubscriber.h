#ifndef MEMENTAR_API_ACTIONSSUBSCRIBER_H
#define MEMENTAR_API_ACTIONSSUBSCRIBER_H

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "Fact.h"
#include "OccasionsSubscriber.h"
#include "mementar/compat/ros.h"

namespace mementar {
  class ActionsSubscriber : private OccasionsSubscriber
  {
  public:
    ActionsSubscriber(const std::function<void(const std::string&)>& callback,
                      const std::string& name = "");

    bool subscribeToStart(const std::string& name, size_t count = -1);
    bool subscribeToEnd(const std::string& name, size_t count = -1);
    bool cancel();

    bool end() { return OccasionsSubscriber::end(); }

  private:
    std::function<void(const std::string&)> callback_;

    void privateCallback(const Fact& fact);
  };

} // namespace mementar

#endif // MEMENTAR_API_ACTIONSSUBSCRIBER_H
