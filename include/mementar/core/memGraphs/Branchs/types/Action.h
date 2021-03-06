#ifndef MEMENTAR_ACTION_H
#define MEMENTAR_ACTION_H

#include "mementar/core/memGraphs/Branchs/ValuedNode.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"

#include <experimental/optional>

namespace mementar {

class ContextualizedFact;

class Action : public ValuedNode
{
public:
  Action(const std::string& name, const SoftPoint& start);
  Action(const std::string& name, const SoftPoint& start, const SoftPoint& end);

  Action(const Action& other) = delete;

  bool setEnd(const SoftPoint& end);

  std::string getName() { return getValue(); }

  size_t getDuration();
  size_t getMinDuration();
  size_t getMaxDuration();

  bool isSoft();
  bool isPending() { return end_ == std::experimental::nullopt; }

  ContextualizedFact* getStartFact() { return start_; }
  ContextualizedFact* getEndFact() { return end_.value(); }

private:
  ContextualizedFact* start_;
  std::experimental::optional<ContextualizedFact*> end_;
};

} // namespace mementar

#endif // MEMENTAR_ACTION_H
