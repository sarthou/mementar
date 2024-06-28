#ifndef MEMENTAR_GRAPH_H
#define MEMENTAR_GRAPH_H

#include <mutex> // For std::unique_lock
#include <shared_mutex>
#include <string>
#include <vector>

#include "mementar/core/memGraphs/BranchContainer/BranchContainerDyn.h"
#include "mementar/core/memGraphs/BranchContainer/BranchContainerMap.h"
#include "mementar/core/memGraphs/Branchs/ValuedNode.h"

namespace mementar {

  template<typename B>
  class Graph
  {
    static_assert(std::is_base_of<ValuedNode, B>::value, "B must be derived from ValuedNode");

  public:
    Graph() = default;
    virtual ~Graph() = default;

    virtual std::vector<B*> get() = 0;
    virtual std::vector<B*> getSafe() = 0;

    virtual B* findBranch(const std::string& name);
    virtual B* findBranchUnsafe(const std::string& name);
    ;

    BranchContainerMap<B> container_;

    std::string language_;

    mutable std::shared_timed_mutex mutex_;
    // use std::lock_guard<std::shared_timed_mutex> lock(Graph<B>::mutex_); to WRITE A DATA
    // use std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_); to READ A DATA
  };

  template<typename B>
  B* Graph<B>::findBranch(const std::string& name)
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return container_.find(name);
  }

  template<typename B>
  B* Graph<B>::findBranchUnsafe(const std::string& name)
  {
    return container_.find(name);
  }

} // namespace mementar

#endif // MEMENTAR_GRAPH_H
