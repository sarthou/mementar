#ifndef MEMENTAR_EVENTLINKEDLEAF_H
#define MEMENTAR_EVENTLINKEDLEAF_H

#include "mementar/core/memGraphs/ExtendedBtree/DlBtree.h"

namespace mementar {

  template<typename Tkey, typename Tdata>
  class EventLinkedLeaf;

  template<typename Tleaf, typename SelfType>
  class LinkedEvent
  {
    template<typename Tkey, typename Tdata>
    friend class EventLinkedLeaf;

  public:
    LinkedEvent() : leaf_(nullptr),
                    prev_elem_(nullptr),
                    next_elem_(nullptr)
    {}

    SelfType* getNext() const { return next_elem_; }
    SelfType* getPrevious() const { return prev_elem_; }

    Tleaf* getLeaf() const
    {
      if(leaf_ == nullptr)
        return nullptr;
      else
        return static_cast<Tleaf*>(leaf_);
    }

    Tleaf* getNextLeaf() const
    {
      if(leaf_ == nullptr)
        return nullptr;
      else
        return static_cast<Tleaf*>(leaf_->getNextLeaf());
    }

    Tleaf* getPreviousLeaf() const
    {
      if(leaf_ == nullptr)
        return nullptr;
      else
        return static_cast<Tleaf*>(leaf_->getPreviousLeaf());
    }

    Tleaf* leaf_;

  protected:
    SelfType* prev_elem_;
    SelfType* next_elem_;

    std::vector<SelfType*> to_link_prev;
    std::vector<SelfType*> to_link_next;
  };

  template<typename Tkey, typename Tdata>
  class EventLinkedLeaf : public DataLinkedLeaf<Tkey, Tdata>
  {
    using DLF = DataLinkedLeaf<Tkey, Tdata>;

  public:
    void insert(typename DLF::LeafType* leaf, Tdata data)
    {
      DLF::insert(leaf, data);
      link(this->payload_.back());
    }

    void remove(typename DLF::LeafType* leaf, Tdata data)
    {
      (void)leaf;
      for(size_t i = 0; i < this->payload_.size();)
      {
        if(this->payload_[i] == data)
        {
          unlink(this->payload_[i]);
          this->payload_[i]->leaf_ = nullptr;
          this->payload_.erase(this->payload_.begin() + i);
        }
        else
          i++;
      }
    }

  private:
    void link(Tdata data);
    void unlink(Tdata current_data);

    Tdata getPrev(Tdata data);
    std::vector<Tdata> getPrevs(Tdata data);

    Tdata getNext(Tdata data);
    std::vector<Tdata> getNexts(Tdata data);

    void linkPrev(Tdata current, Tdata prev, Tdata next);
    void linkNext(Tdata current, Tdata next, Tdata prev);

    void unlinkPrev(Tdata current, const std::vector<Tdata>& prev, Tdata next);
    void unlinkNext(Tdata current, const std::vector<Tdata>& next, Tdata prev);
  };

  template<typename Tkey, typename Tdata>
  void EventLinkedLeaf<Tkey, Tdata>::link(Tdata data)
  {
    auto next = getNext(data);
    auto prev = getPrev(data);

    linkPrev(data, prev, next);
    linkNext(data, next, prev);
  }

  template<typename Tkey, typename Tdata>
  void EventLinkedLeaf<Tkey, Tdata>::unlink(Tdata current_data)
  {
    std::vector<Tdata> prev = getPrevs(current_data);
    std::vector<Tdata> next = getNexts(current_data);

    unlinkPrev(current_data, prev, next.empty() ? nullptr : next[0]);
    unlinkNext(current_data, next, prev.empty() ? nullptr : prev[0]);

    current_data->prev_elem_ = nullptr;
    current_data->next_elem_ = nullptr;

    current_data->to_link_prev.clear();
    current_data->to_link_next.clear();
  }

  template<typename Tkey, typename Tdata>
  Tdata EventLinkedLeaf<Tkey, Tdata>::getPrev(Tdata data)
  {
    Tdata res = nullptr;

    auto prev_node = static_cast<EventLinkedLeaf<Tkey, Tdata>*>(data->getPreviousLeaf());
    while(prev_node != nullptr)
    {
      for(auto& ld : prev_node->payload_)
      {
        if((ld != nullptr) && data->isPartOf(*ld))
          return ld;
      }
      prev_node = static_cast<EventLinkedLeaf<Tkey, Tdata>*>(prev_node->getPreviousLeaf());
    }

    return res;
  }

  template<typename Tkey, typename Tdata>
  std::vector<Tdata> EventLinkedLeaf<Tkey, Tdata>::getPrevs(Tdata data)
  {
    std::vector<Tdata> res;

    auto prev_node = static_cast<EventLinkedLeaf<Tkey, Tdata>*>(data->getPreviousLeaf());
    if(prev_node == nullptr)
      return res;

    for(;;)
    {
      for(auto& ld : prev_node->payload_)
      {
        if(data->isPartOf(*ld))
        {
          res.push_back(ld);
          if(ld->prev_elem_ != nullptr)
            if(ld->prev_elem_->next_elem_->operator==(*ld))
              return res;
          break;
        }
      }

      prev_node = static_cast<EventLinkedLeaf<Tkey, Tdata>*>(prev_node->getPreviousLeaf());
      if(prev_node == nullptr)
        return res;
    }
  }

  template<typename Tkey, typename Tdata>
  Tdata EventLinkedLeaf<Tkey, Tdata>::getNext(Tdata data)
  {
    Tdata res = nullptr;

    auto next_node = static_cast<EventLinkedLeaf<Tkey, Tdata>*>(data->getNextLeaf());
    while(next_node != nullptr)
    {
      for(auto ld : next_node->payload_)
      {
        if((ld != nullptr) && data->isPartOf(*ld))
          return ld;
      }
      next_node = static_cast<EventLinkedLeaf<Tkey, Tdata>*>(next_node->getNextLeaf());
    }

    return res;
  }

  template<typename Tkey, typename Tdata>
  std::vector<Tdata> EventLinkedLeaf<Tkey, Tdata>::getNexts(Tdata data)
  {
    std::vector<Tdata> res;

    auto next_node = static_cast<EventLinkedLeaf<Tkey, Tdata>*>(data->getNextLeaf());
    if(next_node == nullptr)
      return res;

    for(;;)
    {
      for(auto& ld : next_node->payload_)
      {
        if(data->isPartOf(*ld))
        {
          res.push_back(ld);
          if(ld->next_elem_ != nullptr)
            if(ld->next_elem_->prev_elem_->operator==(*ld))
              return res;
          break;
        }
      }

      next_node = static_cast<EventLinkedLeaf<Tkey, Tdata>*>(next_node->getNextLeaf());
      if(next_node == nullptr)
        return res;
    }
  }

  template<typename Tkey, typename Tdata>
  void EventLinkedLeaf<Tkey, Tdata>::linkPrev(Tdata current, Tdata prev, Tdata next)
  {
    if(prev == nullptr)
      return;

    if(current->operator==(*prev))
    {
      if(next == nullptr)
      {
        current->to_link_next = prev->to_link_next;
        prev->to_link_next.clear();
        current->to_link_next.push_back(prev);
      }
      current->prev_elem_ = prev->prev_elem_;
    }
    else
    {
      current->prev_elem_ = prev;
      prev->next_elem_ = current;
      if(prev->to_link_next.empty() == false)
      {
        for(auto& d : prev->to_link_next)
          d->next_elem_ = current;
        prev->to_link_next.clear();
      }
    }
  }

  template<typename Tkey, typename Tdata>
  void EventLinkedLeaf<Tkey, Tdata>::linkNext(Tdata current, Tdata next, Tdata prev)
  {
    if(next == nullptr)
      return;

    if(current->operator==(*next))
    {
      if(prev == nullptr)
      {
        current->to_link_prev = next->to_link_prev;
        next->to_link_prev.clear();
        current->to_link_prev.push_back(next);
      }
      current->next_elem_ = next->next_elem_;
    }
    else
    {
      current->next_elem_ = next;
      next->prev_elem_ = current;
      if(next->to_link_prev.empty() == false)
      {
        for(auto& d : next->to_link_prev)
          d->prev_elem_ = current;
        next->to_link_prev.clear();
      }
    }
  }

  template<typename Tkey, typename Tdata>
  void EventLinkedLeaf<Tkey, Tdata>::unlinkPrev(Tdata current, const std::vector<Tdata>& prev, Tdata next)
  {
    if(prev.empty() == false)
    {
      if(!current->operator==(*prev[0]))
      {
        if((next != nullptr) && !current->operator==(*next))
        {
          for(auto& p : prev)
            p->next_elem_ = current->next_elem_;
        }
        else
        {
          for(auto& p : prev)
            p->next_elem_ = next;
        }
      }
    }
    else if(current->to_link_prev.size())
    {
      if((next != nullptr) && current->operator==(*next))
      {
        next->to_link_prev = std::move(current->to_link_prev);
        next->to_link_prev.pop_back();
      }
    }
  }

  template<typename Tkey, typename Tdata>
  void EventLinkedLeaf<Tkey, Tdata>::unlinkNext(Tdata current, const std::vector<Tdata>& next, Tdata prev)
  {
    if(next.empty() == false)
    {
      if(!current->operator==(*next[0]))
      {
        if((prev != nullptr) && !current->operator==(*prev))
        {
          for(auto& n : next)
            n->prev_elem_ = current->prev_elem_;
        }
        else
        {
          for(auto& n : next)
            n->prev_elem_ = prev;
        }
      }
    }
    else if(current->to_link_next.size())
    {
      if((prev != nullptr) && current->operator==(*prev))
      {
        prev->to_link_next = std::move(current->to_link_next);
        prev->to_link_next.pop_back();
      }
    }
  }

} // namespace mementar

#endif // MEMENTAR_EVENTLINKEDLEAF_H
