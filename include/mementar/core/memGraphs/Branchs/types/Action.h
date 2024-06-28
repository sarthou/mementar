#ifndef MEMENTAR_ACTION_H
#define MEMENTAR_ACTION_H

#include <optional>
#include <string>

#include "mementar/core/memGraphs/Branchs/ValuedNode.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"

namespace mementar {

  class ContextualizedFact;

  class Action : public ValuedNode
  {
  public:
    Action(const std::string& name, const SoftPoint& start);
    Action(const std::string& name, const SoftPoint& start, const SoftPoint& end);
    Action(const Action& other) = delete;

    bool setEnd(const SoftPoint& end);

    std::string getName() const { return getValue(); }

    SoftPoint::Ttime getDuration();
    SoftPoint::Ttime getMinDuration();
    SoftPoint::Ttime getMaxDuration();

    bool isSoft();
    bool isPending() const { return end_ == std::nullopt; }

    ContextualizedFact* getStartFact() const { return start_; }
    ContextualizedFact* getEndFact() { return end_.value(); }

  private:
    ContextualizedFact* start_;
    std::optional<ContextualizedFact*> end_;
  };

} // namespace mementar

#endif // MEMENTAR_ACTION_H
