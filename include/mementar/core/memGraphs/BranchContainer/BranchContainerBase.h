#ifndef MEMENTAR_BRANCHCONTAINERBASE_H
#define MEMENTAR_BRANCHCONTAINERBASE_H

#include <string>
#include <vector>

#include "mementar/core/memGraphs/Branchs/ValuedNode.h"

namespace mementar {

  template<typename B>
  class BranchContainerBase
  {
    static_assert(std::is_base_of<ValuedNode, B>::value, "B must be derived from ValuedNode");

  public:
    BranchContainerBase() = default;
    virtual ~BranchContainerBase() = default;

    virtual B* find(const std::string& word) = 0;
    virtual void load(std::vector<B*>& vect) = 0;
    virtual void insert(B* branch) = 0;
    virtual void erase(B* branch) = 0;

  private:
  };

} // namespace mementar

#endif // MEMENTAR_BRANCHCONTAINERBASE_H
