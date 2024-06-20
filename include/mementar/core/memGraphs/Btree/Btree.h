#ifndef MEMENTAR_BTREE_H
#define MEMENTAR_BTREE_H

#include <iostream>

#include "mementar/core/memGraphs/Btree/BtreeLeafBase.h"
#include "mementar/core/memGraphs/Btree/BtreeNode.h"

namespace mementar {

  template<typename Tkey, typename Tleaf, size_t N = 10>
  class Btree
  {
    using Tdata = typename Tleaf::DataType;

  public:
    Btree() : root_(nullptr),
              first_(nullptr),
              last_(nullptr),
              nb_data_(0)
    {}

    virtual ~Btree()
    {
      delete root_;
    }

    size_t insert(const Tkey& key, const Tdata& data);
    bool remove(const Tkey& key, const Tdata& data);
    BtreeLeafBase<Tkey, Tleaf>* find(const Tkey& key);
    BtreeLeafBase<Tkey, Tleaf>* findNear(const Tkey& key);
    BtreeLeafBase<Tkey, Tleaf>* getFirst() { return first_; }
    BtreeLeafBase<Tkey, Tleaf>* getLast() { return last_; }

    void displayLinear(int count = -1);
    void displayTree();

    size_t size() { return nb_data_; }

  private:
    BtreeNode<Tkey, Tleaf>* root_;
    BtreeLeafBase<Tkey, Tleaf>* first_;
    BtreeLeafBase<Tkey, Tleaf>* last_;

    std::vector<BtreeNode<Tkey, Tleaf>*> nodes_;
    std::vector<BtreeLeafBase<Tkey, Tleaf>*> leafs_;

    size_t nb_data_;

    BtreeLeafBase<Tkey, Tleaf>* insertInLeaf(BtreeNode<Tkey, Tleaf>* node, const Tkey& key, const Tdata& data);
    BtreeLeafBase<Tkey, Tleaf>* insertInNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key, const Tdata& data);
    void insertNode(BtreeNode<Tkey, Tleaf>* node, BtreeNode<Tkey, Tleaf>* new_node, const Tkey& key);
    bool needBalancing(BtreeNode<Tkey, Tleaf>* node);
    void splitLeafNode(BtreeNode<Tkey, Tleaf>* node);
    void splitNode(BtreeNode<Tkey, Tleaf>* node);

    bool removeInNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key, const Tdata& data);
    bool removeInLeafNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key, const Tdata& data);

    BtreeLeafBase<Tkey, Tleaf>* findNearInNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key);
    BtreeLeafBase<Tkey, Tleaf>* findInNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key);
    BtreeLeafBase<Tkey, Tleaf>* findNearInLeafNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key);
    BtreeLeafBase<Tkey, Tleaf>* findInLeafNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key);

    void displayNode(BtreeNode<Tkey, Tleaf>* node, size_t depth = 0);
    void displayLeafNode(BtreeNode<Tkey, Tleaf>* node, size_t depth);
  };

  template<typename Tkey, typename Tleaf, size_t N>
  size_t Btree<Tkey, Tleaf, N>::insert(const Tkey& key, const Tdata& data)
  {
    nb_data_++;
    if(last_ == nullptr) // first leaf in the entire Btree
    {
      root_ = new BtreeNode<Tkey, Tleaf>();
      first_ = insertInLeaf(root_, key, data);
      last_ = first_;
      leafs_.push_back(first_);
      nodes_.push_back(root_);
    }
    else
    {
      BtreeLeafBase<Tkey, Tleaf>* tmp = insertInNode(root_, key, data);
      if(tmp != nullptr)
      {
        leafs_.push_back(tmp);
        if(tmp->operator>(*last_))
          last_ = tmp;
        else if(tmp->operator<(*first_))
          first_ = tmp;

        if(root_->getMother() != nullptr)
          root_ = root_->getMother();
      }
    }

    return nb_data_;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  bool Btree<Tkey, Tleaf, N>::remove(const Tkey& key, const Tdata& data)
  {
    nb_data_--;
    if(root_ != nullptr)
      if(removeInNode(root_, key, data))
      {
        nb_data_--;
        return true;
      }
    return false;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  BtreeLeafBase<Tkey, Tleaf>* Btree<Tkey, Tleaf, N>::findNear(const Tkey& key)
  {
    if(root_ != nullptr)
      return findNearInNode(root_, key);
    else
      return nullptr;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  BtreeLeafBase<Tkey, Tleaf>* Btree<Tkey, Tleaf, N>::find(const Tkey& key)
  {
    if(root_ != nullptr)
      return findInNode(root_, key);
    else
      return nullptr;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  BtreeLeafBase<Tkey, Tleaf>* Btree<Tkey, Tleaf, N>::findNearInNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key)
  {
    if(node->isLeafNode())
      return findNearInLeafNode(node, key);

    for(size_t i = 0; i < node->keys_.size(); i++)
    {
      if(node->keys_[i] > key)
        return findNearInNode(node->childs_[i], key);
    }
    return findNearInNode(node->childs_.back(), key);
  }

  template<typename Tkey, typename Tleaf, size_t N>
  BtreeLeafBase<Tkey, Tleaf>* Btree<Tkey, Tleaf, N>::findInNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key)
  {
    if(node->isLeafNode())
      return findInLeafNode(node, key);

    for(size_t i = 0; i < node->keys_.size(); i++)
    {
      if(node->keys_[i] > key)
        return findInNode(node->childs_[i], key);
    }
    return findInNode(node->childs_.back(), key);
  }

  template<typename Tkey, typename Tleaf, size_t N>
  BtreeLeafBase<Tkey, Tleaf>* Btree<Tkey, Tleaf, N>::findNearInLeafNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key)
  {
    for(size_t i = 0; i < node->keys_.size(); i++)
    {
      if(node->keys_[i] >= key)
        return node->leafs_[i];
    }
    return node->leafs_.back();
  }

  template<typename Tkey, typename Tleaf, size_t N>
  BtreeLeafBase<Tkey, Tleaf>* Btree<Tkey, Tleaf, N>::findInLeafNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key)
  {
    for(size_t i = 0; i < node->keys_.size(); i++)
    {
      if(node->keys_[i] == key)
        return node->leafs_[i];
    }
    return nullptr;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  bool Btree<Tkey, Tleaf, N>::removeInNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key, const Tdata& data)
  {
    if(node->isLeafNode())
      return removeInLeafNode(node, key, data);

    size_t index = node->keys_.size();
    for(size_t i = 0; i < node->keys_.size(); i++)
    {
      if(node->keys_[i] > key)
      {
        index = i;
        break;
      }
    }
    return removeInNode(node->childs_[index], key, data);
  }

  template<typename Tkey, typename Tleaf, size_t N>
  bool Btree<Tkey, Tleaf, N>::removeInLeafNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key, const Tdata& data)
  {
    for(size_t i = 0; i < node->keys_.size(); i++)
    {
      if(node->keys_[i] == key)
      {
        node->leafs_[i]->remove(node->leafs_[i], data);
        if(node->leafs_[i]->hasData() == false)
        {
          if(node->leafs_[i]->getPreviousLeaf() != nullptr)
            node->leafs_[i]->getPreviousLeaf()->setNextLeaf(node->leafs_[i]->getNextLeaf());
          if(node->leafs_[i]->getNextLeaf() != nullptr)
            node->leafs_[i]->getNextLeaf()->setPreviousLeaf(node->leafs_[i]->getPreviousLeaf());

          delete node->leafs_[i];
          node->leafs_.erase(node->leafs_.begin() + i);
          node->keys_.erase(node->keys_.begin() + i);

          if(node->leafs_.size() == 0)
            std::cout << "a node is empty but will not be destroyed" << std::endl;
        }
        return true;
      }
    }
    return false;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  BtreeLeafBase<Tkey, Tleaf>* Btree<Tkey, Tleaf, N>::insertInLeaf(BtreeNode<Tkey, Tleaf>* node, const Tkey& key, const Tdata& data)
  {
    BtreeLeafBase<Tkey, Tleaf>* res = nullptr;

    if(node->leafs_.size() == 0)
    {
      node->keys_.push_back(key);
      res = new BtreeLeafBase<Tkey, Tleaf>(key);
      node->leafs_.push_back(res);
      res->setMother(node);
      res->insert(res, data);
      return res;
    }
    else
    {
      BtreeLeafBase<Tkey, Tleaf>* last = node->leafs_.back();

      if(key > node->keys_.back())
      {
        node->keys_.push_back(key);
        res = new BtreeLeafBase<Tkey, Tleaf>(key);
        node->leafs_.push_back(res);
        res->setNextLeaf(last->getNextLeaf());
        last->setNextLeaf(res);
        res->setPreviousLeaf(last);
        res->setMother(node);
        res->insert(res, data);
      }
      else if(node->keys_.back() == key)
      {
        last->insert(last, data);
      }
      else
      {
        for(size_t i = 0; i < node->keys_.size(); i++)
        {
          if(node->keys_[i] == key)
          {
            node->leafs_[i]->insert(node->leafs_[i], data);
            break;
          }
          else if(node->keys_[i] > key)
          {
            res = new BtreeLeafBase<Tkey, Tleaf>(key);
            // here last is the next node of res
            last = node->leafs_[i];
            res->setNextLeaf(last);
            res->setPreviousLeaf(last->getPreviousLeaf());
            if(res->getPreviousLeaf())
              res->getPreviousLeaf()->setNextLeaf(res);
            if(res->getNextLeaf())
              res->getNextLeaf()->setPreviousLeaf(res);
            node->keys_.insert(node->keys_.begin() + i, key);
            node->leafs_.insert(node->leafs_.begin() + i, res);
            res->setMother(node);
            res->insert(res, data);
            break;
          }
        }
      }
    }

    if(needBalancing(node))
    {
      if(node->isLeafNode())
        splitLeafNode(node);
      else
        splitNode(node);
    }

    return res;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  BtreeLeafBase<Tkey, Tleaf>* Btree<Tkey, Tleaf, N>::insertInNode(BtreeNode<Tkey, Tleaf>* node, const Tkey& key, const Tdata& data)
  {
    if(node->isLeafNode())
      return insertInLeaf(node, key, data);
    else
    {
      size_t index = 0;
      for(index = 0; index < node->keys_.size(); index++)
      {
        if(key < node->keys_[index])
          break;
      }
      return insertInNode(node->childs_[index], key, data);
    }
  }

  template<typename Tkey, typename Tleaf, size_t N>
  void Btree<Tkey, Tleaf, N>::insertNode(BtreeNode<Tkey, Tleaf>* node, BtreeNode<Tkey, Tleaf>* new_node, const Tkey& key)
  {
    if(node->childs_.size() == 0)
    {
      node->childs_.push_back(new_node);
      new_node->setMother(node);
      return;
    }
    else
    {
      if(node->keys_.size() == 0)
      {
        node->keys_.push_back(key);
        node->childs_.push_back(new_node);
        new_node->setMother(node);
      }
      else if(key > node->keys_.back())
      {
        node->keys_.push_back(key);
        node->childs_.push_back(new_node);
        new_node->setMother(node);
      }
      else
      {
        for(size_t i = 0; i < node->keys_.size(); i++)
        {
          if(key < node->keys_[i])
          {
            node->keys_.insert(node->keys_.begin() + i, key);
            node->childs_.insert(node->childs_.begin() + i + 1, new_node);
            new_node->setMother(node);
            break;
          }
        }
      }
    }

    if(needBalancing(node))
      splitNode(node);
  }

  template<typename Tkey, typename Tleaf, size_t N>
  bool Btree<Tkey, Tleaf, N>::needBalancing(BtreeNode<Tkey, Tleaf>* node)
  {
    if(node->isLeafNode())
      return node->leafs_.size() > N;
    else
      return node->childs_.size() > N;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  void Btree<Tkey, Tleaf, N>::splitLeafNode(BtreeNode<Tkey, Tleaf>* node)
  {
    BtreeNode<Tkey, Tleaf>* new_leaf_node = new BtreeNode<Tkey, Tleaf>();

    size_t half_order = N / 2;
    for(size_t i = 0; i < half_order; i++)
    {
      new_leaf_node->leafs_.insert(new_leaf_node->leafs_.begin(), node->leafs_.back());
      node->leafs_.pop_back();
      new_leaf_node->leafs_[i]->setMother(new_leaf_node);

      new_leaf_node->keys_.insert(new_leaf_node->keys_.begin(), node->keys_.back());
      node->keys_.pop_back();
    }

    if(node->mother_ != nullptr)
    {
      insertNode(node->mother_, new_leaf_node, new_leaf_node->keys_[0]);
    }
    else
    {
      BtreeNode<Tkey, Tleaf>* new_mother = new BtreeNode<Tkey, Tleaf>();
      nodes_.push_back(new_mother);
      insertNode(new_mother, node, node->keys_[0]);
      insertNode(new_mother, new_leaf_node, new_leaf_node->keys_[0]);
    }
  }

  template<typename Tkey, typename Tleaf, size_t N>
  void Btree<Tkey, Tleaf, N>::splitNode(BtreeNode<Tkey, Tleaf>* node)
  {
    BtreeNode<Tkey, Tleaf>* new_node = new BtreeNode<Tkey, Tleaf>();
    nodes_.push_back(new_node);

    size_t half_order = N / 2 - 1;
    for(size_t i = 0; i < half_order; i++)
    {
      new_node->childs_.insert(new_node->childs_.begin(), node->childs_.back());
      node->childs_.pop_back();
      new_node->childs_[i]->setMother(new_node);

      new_node->keys_.insert(new_node->keys_.begin(), node->keys_.back());
      node->keys_.pop_back();
    }

    new_node->childs_.insert(new_node->childs_.begin(), node->childs_.back());
    node->childs_.pop_back();
    new_node->childs_[half_order]->setMother(new_node);

    if(node->mother_ != nullptr)
    {
      insertNode(node->mother_, new_node, node->keys_.back());
      node->keys_.pop_back();
    }
    else
    {
      BtreeNode<Tkey, Tleaf>* new_mother = new BtreeNode<Tkey, Tleaf>();
      nodes_.push_back(new_mother);
      insertNode(new_mother, node, node->keys_.back());
      insertNode(new_mother, new_node, node->keys_.back());
      node->keys_.pop_back();
    }
  }

  template<typename Tkey, typename Tleaf, size_t N>
  void Btree<Tkey, Tleaf, N>::displayLinear(int count)
  {
    BtreeLeafBase<Tkey, Tleaf>* tmp = first_;
    int cpt = 0;
    std::cout << "******" << std::endl;
    while((tmp != nullptr) && ((cpt < count) || (count == -1)))
    {
      std::cout << tmp->getKey() << " => ";
      for(auto data : tmp->getData())
        std::cout << data << " : ";
      std::cout << std::endl;
      tmp = tmp->getNextLeaf();
      cpt++;
    }
    std::cout << "******" << std::endl;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  void Btree<Tkey, Tleaf, N>::displayTree()
  {
    std::cout << "******" << std::endl;
    if(root_)
      displayNode(root_);
    std::cout << "******" << std::endl;
  }

  template<typename Tkey, typename Tleaf, size_t N>
  void Btree<Tkey, Tleaf, N>::displayNode(BtreeNode<Tkey, Tleaf>* node, size_t depth)
  {
    size_t depth_1 = depth + 1;

    if(node->isLeafNode())
      displayLeafNode(node, depth_1);
    else
    {
      for(size_t i = 0; i < node->keys_.size(); i++)
      {
        displayNode(node->childs_[i], depth_1);
        for(size_t j = 0; j < depth; j++)
          std::cout << "\t";
        std::cout << node->keys_[i] << std::endl;
      }
      displayNode(node->childs_[node->keys_.size()], depth_1);
    }
  }

  template<typename Tkey, typename Tleaf, size_t N>
  void Btree<Tkey, Tleaf, N>::displayLeafNode(BtreeNode<Tkey, Tleaf>* node, size_t depth)
  {
    for(size_t i = 0; i < node->keys_.size(); i++)
    {
      for(size_t j = 0; j < depth; j++)
        std::cout << "\t";
      std::cout << node->keys_[i] << " => ";
      for(auto data : node->leafs_[i]->getData())
        std::cout << data << " : ";
      std::cout << std::endl;
    }
  }

} // namespace mementar

#endif // MEMENTAR_BTREE_H
