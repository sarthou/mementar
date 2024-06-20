#ifndef MEMENTAR_BRANCHCONTAINERDYN_H
#define MEMENTAR_BRANCHCONTAINERDYN_H

#include <iostream>
#include <map>
#include <math.h>

#include "mementar/core/memGraphs/BranchContainer/BranchContainerBase.h"

namespace mementar {

  template<typename T>
  struct BranchNode_t
  {
    BranchNode_t<T>* next;
    std::string id;
    T* branch;

    BranchNode_t() : next(nullptr), branch(nullptr) {}
    ~BranchNode_t()
    {
      // T* branch is destructed by ontograph
      if(next != nullptr)
      {
        delete next;
        next = nullptr;
      }
    }
  };

  template<typename B>
  class BranchContainerDyn : public BranchContainerBase<B>
  {
  public:
    BranchContainerDyn() : nodes_(new BranchNode_t<B>),
                           nodes_end_(nodes_),
                           buffer_size_(0), nb_elem_(0)
    {}
    BranchContainerDyn(const BranchContainerDyn& base);

    virtual ~BranchContainerDyn()
    {
      delete nodes_;
    }

    virtual B* find(const std::string& word);
    virtual void load(std::vector<B*>& vect);
    virtual void insert(B* branch);
    virtual void erase(B* branch);

  private:
    BranchNode_t<B>* nodes_;
    BranchNode_t<B>* nodes_end_;
    size_t buffer_size_;
    size_t nb_elem_;

    void insertEnd(const std::string& id, B* branch);
    void reconf(BranchNode_t<B>* node);
  };

  template<typename B>
  BranchContainerDyn<B>::BranchContainerDyn(const BranchContainerDyn& base)
  {
    BranchNode_t<B>* current = base.nodes_;
    while(current != nullptr)
    {
      B* tmp = new B();
      *tmp = *(current->branch);
      insertEnd(current->id, tmp);
    }

    buffer_size_ = base.buffer_size_;
    nb_elem_ = base.nb_elem_;
  }

  template<typename B>
  B* BranchContainerDyn<B>::find(const std::string& word)
  {
    B* tmp = nullptr;
    size_t i = 0;
    for(BranchNode_t<B>* node = nodes_; node->next != nullptr; node = node->next)
    {
      if(node->next->id == word)
      {
        tmp = node->next->branch;
        if(i > 5)
          reconf(node);
        break;
      }
      i++;
    }

    return tmp;
  }

  template<typename B>
  void BranchContainerDyn<B>::load(std::vector<B*>& vect)
  {
    for(size_t i = 0; i < vect.size(); i++)
      insertEnd(vect[i]->getValue(), vect[i]);

    nb_elem_ += vect.size();
    buffer_size_ = log2(nb_elem_);
  }

  template<typename B>
  void BranchContainerDyn<B>::insert(B* branch)
  {
    insertEnd(branch->getValue(), branch);
  }

  template<typename B>
  void BranchContainerDyn<B>::erase(B* branch)
  {
    for(BranchNode_t<B>* node = nodes_; node->next != nullptr; node = node->next)
      if(node->next->branch == branch)
      {
        BranchNode_t<B>* tmp = node->next;
        node->next = tmp->next;
        delete (tmp);
        break;
      }
  }

  template<typename B>
  void BranchContainerDyn<B>::insertEnd(const std::string& id, B* branch)
  {
    BranchNode_t<B>* tmp = new BranchNode_t<B>;
    tmp->id = id;
    tmp->branch = branch;
    nodes_end_->next = tmp;
    nodes_end_ = tmp;
  }

  template<typename B>
  void BranchContainerDyn<B>::reconf(BranchNode_t<B>* node)
  {
    BranchNode_t<B>* tmp = node->next;
    node->next = tmp->next;
    tmp->next = nodes_->next;
    nodes_->next = tmp;
  }

} // namespace mementar

#endif // MEMENTAR_BRANCHCONTAINERDYN_H
