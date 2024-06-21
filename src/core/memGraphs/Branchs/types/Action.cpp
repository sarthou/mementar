#include "mementar/core/memGraphs/Branchs/types/Action.h"

#include <cstddef>
#include <optional>
#include <string>

#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"
#include "mementar/core/memGraphs/Branchs/ValuedNode.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"

namespace mementar {
  Action::Action(const std::string& name, const SoftPoint& start) : ValuedNode(name)
  {
    start_ = new ContextualizedFact(std::string(name + "_start"), Fact(name + "|_|start", start), this);
    end_ = std::nullopt;
  }

  Action::Action(const std::string& name, const SoftPoint& start, const SoftPoint& end) : ValuedNode(name)
  {
    start_ = new ContextualizedFact(std::string(name + "_start"), Fact(name + "|_|start", start), this);
    end_ = new ContextualizedFact(std::string(name + "_end"), Fact(name + "|_|end", end), this);
  }

  bool Action::setEnd(const SoftPoint& end)
  {
    if(end_ != std::nullopt)
      return false;

    end_ = new ContextualizedFact(std::string(getValue() + "_end"), Fact(getValue() + "|_|end", end), this);
    return true;
  }

  size_t Action::getDuration()
  {
    return (end_ ? end_.value()->getTime() - start_->getTime() : 0);
  }

  size_t Action::getMinDuration()
  {
    return (end_ ? end_.value()->getTimeStart() - start_->getTimeEnd() : 0);
  }

  size_t Action::getMaxDuration()
  {
    return (end_ ? end_.value()->getTimeEnd() - start_->getTimeStart() : 0);
  }

  bool Action::isSoft()
  {
    if(end_)
      return ((start_->isInstantaneous() && end_.value()->isInstantaneous()) == false);
    else
      return (start_->isInstantaneous() == false);
  }

} // namespace mementar
