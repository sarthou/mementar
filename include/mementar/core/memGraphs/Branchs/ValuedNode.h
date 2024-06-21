#ifndef MEMENTAR_VALUEDNODE_H
#define MEMENTAR_VALUEDNODE_H

#include <string>

#include "mementar/core/memGraphs/Branchs/WordTable.h"

namespace mementar {

  class ValuedNode
  {
  public:
    explicit ValuedNode(const std::string& value) : index_(table_.add(value))
    {}

    uint32_t get() const { return index_; }
    std::string getValue() const { return table_[index_]; }

    static WordTable table_;

  private:
    uint32_t index_;
  };

} // namespace mementar

#endif // MEMENTAR_VALUEDNODE_H
