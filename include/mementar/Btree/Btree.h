#ifndef MEMENTAR_BTREE_H
#define MEMENTAR_BTREE_H

#include <iostream>
#include <math.h>

#include "mementar/Btree/BtreeNode.h"
#include "mementar/Btree/BtreeLeafNode.h"
#include "mementar/Btree/BtreeLeaf.h"

namespace mementar
{

template<typename Tkey, typename Tdata>
class Btree
{
public:
  Btree(size_t order = 10)
  {
    root_ = nullptr;
    last_ = nullptr;
    order_ = order;
    level_ = 0;
    nb_data_ = 0;
  }

  ~Btree()
  {
    if(root_ != nullptr)
      delete root_;

    BtreeLeaf<Tkey, Tdata>* tmp;
    while(last_ != nullptr)
    {
      tmp = last_;
      last_ = last_->prev_;
      delete tmp;
    }
  }

  size_t insert(const Tkey& key, const Tdata& data);
  bool remove(const Tkey& key, const Tdata& data);
  BtreeLeaf<Tkey, Tdata>* find(const Tkey& key);
  BtreeLeaf<Tkey, Tdata>* findNear(const Tkey& key);
  BtreeLeaf<Tkey, Tdata>* getFirst();
  BtreeLeaf<Tkey, Tdata>* getLast() { return last_; }

  size_t estimateMinLeaves()
  {
    return pow((double)order_/2. + 1., (double)level_ - 1.) * root_->getNbChilds();
  }

  size_t estimateMaxLevel(size_t nbLeafs)
  {
    return log((double)nbLeafs/2.) / log((double)order_/2. + 1) + 1;
  }

  size_t getCurrentLevel() { return level_; }

  void display(int count = -1);

private:
  BtreeNode<Tkey, Tdata>* root_;
  BtreeLeaf<Tkey, Tdata>* last_;
  size_t order_;
  size_t level_;
  size_t nb_data_;
};

template<typename Tkey, typename Tdata>
size_t Btree<Tkey,Tdata>::insert(const Tkey& key, const Tdata& data)
{
  nb_data_++;
  if(last_ == nullptr)
  {
    root_ = new BtreeLeafNode<Tkey, Tdata>(order_);
    level_ = root_->getLevel();
    last_ = root_->insert(key, data);
  }
  else
  {
    BtreeLeaf<Tkey, Tdata>* tmp = root_->insert(key, data);
    if(tmp != nullptr)
    {
      if(tmp->operator>(last_))
        last_ = tmp;
      if(root_->getMother() != nullptr)
      {
        root_ = root_->getMother();
        level_ = root_->getLevel();
      }
    }
  }

  return nb_data_;
}

template<typename Tkey, typename Tdata>
bool Btree<Tkey,Tdata>::remove(const Tkey& key, const Tdata& data)
{
  nb_data_--;
  if(root_ != nullptr)
    return root_->remove(key, data);
  return false;
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* Btree<Tkey,Tdata>::find(const Tkey& key)
{
  if(root_ != nullptr)
    return root_->find(key);
  else
    return nullptr;
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* Btree<Tkey,Tdata>::findNear(const Tkey& key)
{
  if(root_ != nullptr)
    return root_->findNear(key);
  else
    return nullptr;
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* Btree<Tkey,Tdata>::getFirst()
{
  if(root_ != nullptr)
    return root_->getFirst();
  else
    return nullptr;
}

template<typename Tkey, typename Tdata>
void Btree<Tkey,Tdata>::display(int count)
{
  BtreeLeaf<Tkey, Tdata>* tmp = last_;
  int cpt = 0;
  std::cout << "******" << std::endl;
  while((tmp != nullptr) && ((cpt < count) || (count == -1)))
  {
    std::vector<Tdata> datas = tmp->getData();
    std::cout << tmp->getKey() << " => ";
    for(const auto& data : datas)
      std::cout << data << " : ";
    std::cout << std::endl;
    tmp = tmp->prev_;
    cpt++;
  }
  std::cout << "******" << std::endl;
  root_->display();
}

} // namespace mementar

#endif // MEMENTAR_BTREE_H
