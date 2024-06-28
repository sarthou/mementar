#ifndef MEMENTAR_BPLUSTREE_H
#define MEMENTAR_BPLUSTREE_H

#include "mementar/core/memGraphs/Btree/Btree.h"

namespace mementar {

  template<typename Tkey, typename Tdata>
  class BplusLeaf
  {
  public:
    using LeafType = BtreeLeafBase<Tkey, BplusLeaf<Tkey, Tdata>>;
    using DataType = Tdata;

    BplusLeaf() : prev_(nullptr),
                  next_(nullptr)
    {}

    void insert(LeafType* leaf, DataType data)
    {
      (void)leaf;
      payload_.push_back(data);
    }
    void remove(LeafType* leaf, DataType data)
    {
      (void)leaf;
      for(size_t i = 0; i < payload_.size();)
      {
        if(payload_[i] == data)
          payload_.erase(payload_.begin() + i);
        else
          i++;
      }
    }
    bool hasData() const
    {
      return (payload_.empty() == false);
    }

    std::vector<DataType> getData() const
    {
      return payload_;
    }

    BplusLeaf* getPreviousLeaf() const { return prev_; }
    BplusLeaf* getNextLeaf() const { return next_; }

    void setPreviousLeaf(BplusLeaf* prev) { prev_ = prev; }
    void setNextLeaf(BplusLeaf* next) { next_ = next; }

    std::vector<DataType> payload_;
    BplusLeaf* prev_;
    BplusLeaf* next_;

  private:
  };

  template<typename Tkey, typename Tdata>
  class BplusLeaf<Tkey, Tdata*>
  {
  public:
    using LeafType = BtreeLeafBase<Tkey, BplusLeaf<Tkey, Tdata*>>;
    using DataType = Tdata*;

    BplusLeaf() : prev_(nullptr),
                  next_(nullptr)
    {}

    void insert(LeafType* leaf, DataType data)
    {
      (void)leaf;
      payload_.push_back(data);
    }
    void remove(LeafType* leaf, DataType data)
    {
      (void)leaf;
      for(size_t i = 0; i < payload_.size();)
      {
        if(payload_[i]->operator==(*data))
          payload_.erase(payload_.begin() + i);
        else
          i++;
      }
    }
    bool hasData()
    {
      return (payload_.empty() == false);
    }

    std::vector<DataType> getData() const
    {
      return payload_;
    }

    BplusLeaf* getPreviousLeaf() const { return prev_; }
    BplusLeaf* getNextLeaf() const { return next_; }

    void setPreviousLeaf(BplusLeaf* prev) { prev_ = prev; }
    void setNextLeaf(BplusLeaf* next) { next_ = next; }

    std::vector<DataType> payload_;
    BplusLeaf* prev_;
    BplusLeaf* next_;

  private:
  };

  template<typename Tkey, typename Tdata, size_t N = 3>
  class BplusTree : public Btree<Tkey, BplusLeaf<Tkey, Tdata>, N>
  {
  public:
    template<typename T>
    class BtreeIterator
    {
    public:
      using iterator_category = std::bidirectional_iterator_tag;
      using value_type = T;
      using pointer = T*;
      using reference = T&;

      BtreeIterator(T* ptr = nullptr) : ptr_(ptr) {}
      BtreeIterator(const BtreeIterator<T>& other) = default;

      BtreeIterator<T>& operator=(const BtreeIterator<T>& other) = default;
      BtreeIterator<T>& operator=(BtreeIterator<T>* other)
      {
        ptr_ = other->ptr;
        return (*this);
      }

      operator bool() const
      {
        if(ptr_)
          return true;
        else
          return false;
      }

      bool operator==(const BtreeIterator<T>& other) const { return (ptr_ == other.ptr_); }
      bool operator!=(const BtreeIterator<T>& other) const { return (ptr_ != other.ptr_); }

      BtreeIterator<T>& operator++()
      {
        ptr_ = ptr_->getNextLeaf();
        return (*this);
      }
      BtreeIterator<T>& operator--()
      {
        ptr_ = ptr_->getPreviousLeaf();
        return (*this);
      }

      T& operator*() { return *ptr_; }
      const T& operator*() const { return *ptr_; }
      T* operator->() { return ptr_; }

    protected:
      T* ptr_;
    };

    template<typename T>
    class BtreeReverseIterator : public BtreeIterator<T>
    {
    public:
      BtreeReverseIterator(T* ptr = nullptr) : BtreeIterator<T>(ptr) {}
      explicit BtreeReverseIterator(const BtreeIterator<T>& forward_iterator) { this->ptr_ = *forward_iterator; }
      BtreeReverseIterator(const BtreeReverseIterator<T>& other) = default;

      BtreeReverseIterator<T>& operator=(const BtreeReverseIterator<T>& other) = default;
      BtreeReverseIterator<T>& operator=(const BtreeIterator<T>& forward_iterator)
      {
        this->ptr_ = *forward_iterator;
        return (*this);
      }
      BtreeReverseIterator<T>& operator=(T* ptr)
      {
        this->ptr_ = ptr;
        return (*this);
      }

      BtreeReverseIterator<T>& operator++()
      {
        this->ptr_ = this->ptr_->getPreviousLeaf();
        return (*this);
      }
      BtreeReverseIterator<T>& operator--()
      {
        this->ptr_ = this->ptr_->getNextLeaf();
        return (*this);
      }

      BtreeIterator<T> base()
      {
        BtreeIterator<T> forward_iterator(this->ptr_);
        ++forward_iterator;
        return forward_iterator;
      }
    };

    using iterator = BtreeIterator<BplusLeaf<Tkey, Tdata>>;
    using const_iterator = BtreeIterator<const BplusLeaf<Tkey, Tdata>>;

    using reverse_iterator = BtreeReverseIterator<BplusLeaf<Tkey, Tdata>>;
    using const_reverse_iterator = BtreeReverseIterator<const BplusLeaf<Tkey, Tdata>>;

    iterator begin() { return iterator(this->getFirst()); }
    iterator end() { return iterator(nullptr); }

    const_iterator cbegin() { return const_iterator(this->getFirst()); }
    const_iterator cend() { return const_iterator(nullptr); }

    reverse_iterator rbegin() { return reverse_iterator(this->getLast()); }
    reverse_iterator rend() { return reverse_iterator(nullptr); }

    const_reverse_iterator crbegin() { return const_reverse_iterator(this->getLast()); }
    const_reverse_iterator crend() { return const_reverse_iterator(nullptr); }
  };

} // namespace mementar

#endif // MEMENTAR_BPLUSTREE_H
