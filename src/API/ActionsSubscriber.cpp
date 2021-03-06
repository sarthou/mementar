#include "mementar/API/ActionsSubscriber.h"

namespace mementar
{

ActionsSubscriber::ActionsSubscriber(std::function<void(const std::string&)> callback,
                                     const std::string& name,
                                    bool spin_thread) : OccasionsSubscriber([this](const Fact& fact){ this->privateCallback(fact); }, name, spin_thread)
{
  callback_ = callback;
}

ActionsSubscriber::ActionsSubscriber(std::function<void(const std::string&)> callback,
                                     bool spin_thread) : OccasionsSubscriber([this](const Fact& fact){ this->privateCallback(fact); }, spin_thread)
{
  callback_ = callback;
}

ActionsSubscriber::~ActionsSubscriber()
{
}

bool ActionsSubscriber::subscribeToStart(const std::string& name, size_t count)
{
  return subscribe({name, "_", "start"}, count);
}

bool ActionsSubscriber::subscribeToEnd(const std::string& name, size_t count)
{
  return subscribe({name, "_", "end"}, count);
}

bool ActionsSubscriber::cancel()
{
  return OccasionsSubscriber::cancel();
}

void ActionsSubscriber::privateCallback(const Fact& fact)
{
  callback_(fact.getSubject());
}

} // namespace mementar
