#ifndef MEMENTAR_LINKEDBTREELEAFNODE_H
#define MEMENTAR_LINKEDBTREELEAFNODE_H

#include "mementar/core/Btree/BtreeLeafNode.h"
#include "mementar/core/LinkedFact.h"

namespace mementar
{

template<typename Tkey>
class LinkedBtreeLeafNode : public BtreeLeafNode<Tkey, LinkedFact>
{
public:
  LinkedBtreeLeafNode(size_t order = 10) : BtreeLeafNode<Tkey,LinkedFact>(order)
  {}

  BtreeLeaf<Tkey,LinkedFact>* insert(const Tkey& key, LinkedFact* data);
  bool remove(const Tkey& key, LinkedFact* data);

private:
  void split();

  LinkedFact* getPrev(BtreeLeaf<Tkey,LinkedFact>* current, LinkedFact* data);
  LinkedFact* getNext(BtreeLeaf<Tkey,LinkedFact>* current, LinkedFact* data);

  void linkPrev(LinkedFact* current, LinkedFact* prev, LinkedFact* next);
  void linkNext(LinkedFact* current, LinkedFact* next, LinkedFact* prev);
};

template<typename Tkey>
BtreeLeaf<Tkey,LinkedFact>* LinkedBtreeLeafNode<Tkey>::insert(const Tkey& key, LinkedFact* data)
{
  BtreeLeaf<Tkey,LinkedFact>* res = BtreeLeafNode<Tkey,LinkedFact>::insert(key, data);

  if(res != nullptr)
  {
    std::cout << "insert using LinkedBtreeLeafNode" << std::endl;
    data->next_ = getNext(res, data);
    data->prev_ = getPrev(res, data);
  }

  return res;
}

template<typename Tkey>
bool LinkedBtreeLeafNode<Tkey>::remove(const Tkey& key, LinkedFact* data)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] == key)
    {
      this->leafs_[i]->remove(data);
      if(this->leafs_[i]->getData().size() == 0)
      {
        if(this->leafs_[i]->prev_ != nullptr)
          this->leafs_[i]->prev_->next_ = this->leafs_[i]->next_;
        if(this->leafs_[i]->next_ != nullptr)
          this->leafs_[i]->next_->prev_ = this->leafs_[i]->prev_;
        delete this->leafs_[i];
        this->leafs_.erase(this->leafs_.begin() + i);
        this->keys_.erase(this->keys_.begin() + i);

        if(this->leafs_.size() == 0)
          std::cout << "a node is empty but will not be destroyed" << std::endl;
      }
      return true;
    }
  }
  return false;
}

template<typename Tkey>
void LinkedBtreeLeafNode<Tkey>::split()
{
  LinkedBtreeLeafNode<Tkey>* new_node = new LinkedBtreeLeafNode<Tkey>(this->order_);

  size_t half_order = this->order_/2;
  for(size_t i = 0; i < half_order; i++)
  {
    new_node->leafs_.insert(new_node->leafs_.begin(), this->leafs_[this->leafs_.size() - 1]);
    this->leafs_.erase(this->leafs_.begin() + this->leafs_.size() - 1);
    new_node->leafs_[i]->setMother(new_node);

    new_node->keys_.insert(new_node->keys_.begin(), this->keys_[this->keys_.size() - 1]);
    this->keys_.erase(this->keys_.begin() + this->keys_.size() - 1);
  }

  if(this->mother_ != nullptr)
  {
    this->mother_->insert(new_node, new_node->keys_[0]);
  }
  else
  {
    BtreeNode<Tkey,LinkedFact>* new_mother = new BtreeNode<Tkey,LinkedFact>(this->order_);
    new_mother->setLevel(this->level_ + 1);
    new_mother->insert(this, new_node->keys_[0]);
    new_mother->insert(new_node, new_node->keys_[0]);
  }
}

template<typename Tkey>
LinkedFact* LinkedBtreeLeafNode<Tkey>::getPrev(BtreeLeaf<Tkey,LinkedFact>* current, LinkedFact* data)
{
  LinkedFact* res = nullptr;

  BtreeLeaf<Tkey,LinkedFact>* leaf = current;
  while(leaf->prev_ != nullptr)
  {
    for(auto data : leaf->getData())
    {
      if(data->isEventPart(*data))
      {
        res = data;
        break;
      }
    }
    leaf = leaf->prev_;
  }

  return res;
}

template<typename Tkey>
LinkedFact* LinkedBtreeLeafNode<Tkey>::getNext(BtreeLeaf<Tkey,LinkedFact>* current, LinkedFact* data)
{
  LinkedFact* res = nullptr;

  BtreeLeaf<Tkey,LinkedFact>* leaf = current;
  while(leaf->next_ != nullptr)
  {
    for(auto data : leaf->getData())
    {
      if(data->isEventPart(*data))
      {
        res = data;
        break;
      }
    }
    leaf = leaf->next_;
  }

  return res;
}

template<typename Tkey>
void LinkedBtreeLeafNode<Tkey>::linkPrev(LinkedFact* current, LinkedFact* prev, LinkedFact* next)
{
  if(current->operator==(*prev))
  {
    if(next == nullptr)
    {
      current->toLinkNext = prev->toLinkNext;
      prev->toLinkNext.clear();
      current->toLinkNext.push_back(prev);
    }
    current->prev_ = prev->prev_;
  }
  else
  {
    current->prev_ = prev;
    prev->next_ = current;
    if(prev->toLinkNext.size())
    {
      for(auto d : prev->toLinkNext)
        d->next_ = current;
      prev->toLinkNext.clear();
    }
  }
}

template<typename Tkey>
void LinkedBtreeLeafNode<Tkey>::linkNext(LinkedFact* current, LinkedFact* next, LinkedFact* prev)
{
  if(current->operator==(*next))
  {
    if(prev == nullptr)
    {
      current->toLinkPrev = next->toLinkPrev;
      next->toLinkPrev.clear();
      current->toLinkPrev.push_back(next);
    }
    current->next_ = prev->next_;
  }
  else
  {
    current->next_ = next;
    prev->prev_ = current;
    if(prev->toLinkPrev.size())
    {
      for(auto d : next->toLinkPrev)
        d->prev_ = current;
      next->toLinkPrev.clear();
    }
  }
}

} // namespace mementar

#endif // MEMENTAR_LINKEDBTREELEAFNODE_H
