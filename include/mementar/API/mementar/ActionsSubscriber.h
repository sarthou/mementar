#ifndef MEMENTAR_API_ACTIONSSUBSCRIBER_H
#define MEMENTAR_API_ACTIONSSUBSCRIBER_H

#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "mementar/compat/ros.h"
#include "Fact.h"
#include "OccasionsSubscriber.h"

namespace mementar {
    class ActionsSubscriber : private OccasionsSubscriber {
    public:
        ActionsSubscriber(std::function<void(const std::string &)> callback, const std::string &name = "",
                          bool spin_thread = true);
        ActionsSubscriber(std::function<void(const std::string &)> callback, bool spin_thread);

        ~ActionsSubscriber();

        bool subscribeToStart(const std::string &name, size_t count = -1);
        bool subscribeToEnd(const std::string &name, size_t count = -1);
        bool cancel();

        bool end() { return OccasionsSubscriber::end(); }

    private:
        std::function<void(const std::string &)> callback_;

        void privateCallback(const Fact &fact);
    };

} // namespace mementar

#endif // MEMENTAR_API_ACTIONSSUBSCRIBER_H
