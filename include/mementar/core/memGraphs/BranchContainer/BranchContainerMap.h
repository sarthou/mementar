#ifndef MEMENTAR_BRANCHCONTAINERMAP_H
#define MEMENTAR_BRANCHCONTAINERMAP_H

#include <map>
#include <unordered_map>

#include "mementar/core/memGraphs/BranchContainer/BranchContainerBase.h"

namespace mementar {

  template<typename B>
  class BranchContainerMap : public BranchContainerBase<B>
  {
  public:
    BranchContainerMap() = default;
    BranchContainerMap(const BranchContainerMap& base) = delete;
    ~BranchContainerMap() override = default; // B* must by destructed by the owner

    B* find(const std::string& word) override;
    void load(std::vector<B*>& vect) override;
    void insert(B* branch) override;
    void erase(B* branch) override;

  private:
    std::unordered_map<std::string, B*> nodes_;
  };

  /*template <typename B>
  BranchContainerMap<B>::BranchContainerMap(const BranchContainerMap& base) = delete;
  {
    for(auto& it : base.nodes_)
    {
      B* tmp = new B();
      *tmp = *(it.second);
      nodes_[it.first] = tmp;
    }
  }*/

  template<typename B>
  B* BranchContainerMap<B>::find(const std::string& word)
  {
    typename std::unordered_map<std::string, B*>::iterator it = nodes_.find(word);
    if(it == nodes_.end())
      return nullptr;
    else
      return it->second;
  }

  template<typename B>
  void BranchContainerMap<B>::load(std::vector<B*>& vect)
  {
    for(size_t i = 0; i < vect.size(); i++)
      nodes_[vect[i]->getValue()] = vect[i];
  }

  template<typename B>
  void BranchContainerMap<B>::insert(B* branch)
  {
    nodes_[branch->getValue()] = branch;
  }

  template<typename B>
  void BranchContainerMap<B>::erase(B* branch)
  {
    nodes_.erase(branch->getValue());
  }

} // namespace mementar

#endif // MEMENTAR_BRANCHCONTAINERMAP_H
