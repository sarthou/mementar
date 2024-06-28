#ifndef MEMENTAR_DLBTREE_H
#define MEMENTAR_DLBTREE_H

#include <type_traits>

#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  template<typename Tvalue, typename Tleaf>
  class LinkedData
  {
  public:
    using LeafType = Tleaf;

    explicit LinkedData(Tvalue value) : value_(value),
                                        leaf_(nullptr)
    {}

    LeafType* getNextLeaf() const
    {
      if(leaf_ == nullptr)
        return nullptr;
      else
        return static_cast<LeafType*>(leaf_->getNextLeaf());
    }

    LeafType* getPreviousLeaf() const
    {
      if(leaf_ == nullptr)
        return nullptr;
      else
        return static_cast<LeafType*>(leaf_->getPreviousLeaf());
    }

    friend std::ostream& operator<<(std::ostream& os, const LinkedData& data)
    {
      os << data.value_;
      return os;
    }

    bool operator==(const LinkedData& other) const
    {
      return value_ == other.value_;
    }

    Tvalue value_;
    LeafType* leaf_;
  };

  template<typename Tkey, typename Tdata>
  class DataLinkedLeaf : public BplusLeaf<Tkey, Tdata>
  {
  public:
    using LeafType = typename Tdata::LeafType;

    void insert(LeafType* leaf, Tdata data)
    {
      this->payload_.emplace_back(data);
      this->payload_.back().leaf_ = leaf;
    }

    void remove(LeafType* leaf, Tdata data)
    {
      (void)leaf;
      for(size_t i = 0; i < this->payload_.size();)
      {
        if(this->payload_[i] == data)
        {
          this->payload_[i].leaf_ = nullptr;
          this->payload_.erase(this->payload_.begin() + i);
        }
        else
          i++;
      }
    }
  };

  template<typename Tkey, typename Tdata>
  class DataLinkedLeaf<Tkey, Tdata*> : public BplusLeaf<Tkey, Tdata*>
  {
  public:
    using LeafType = typename Tdata::LeafType;

    void insert(LeafType* leaf, Tdata* data)
    {
      this->payload_.emplace_back(data);
      data->leaf_ = leaf;
    }
    void remove(LeafType* leaf, Tdata* data)
    {
      (void)leaf;
      for(size_t i = 0; i < this->payload_.size();)
      {
        if(this->payload_[i] == *data)
        {
          this->payload_[i].leaf_ = nullptr;
          this->payload_.erase(this->payload_.begin() + i);
        }
        else
          i++;
      }
    }
  };

  template<typename Tkey, typename Tvalue>
  class DlLeaf : public DataLinkedLeaf<Tkey, LinkedData<Tvalue, DlLeaf<Tkey, Tvalue>>>
  {
  };

  template<typename Tkey, typename Tdata, size_t N = 3>
  using DlBtree = Btree<Tkey, DlLeaf<Tkey, Tdata>, N>;

} // namespace mementar

#endif // MEMENTAR_DLBTREE_H
