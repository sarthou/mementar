#include "mementar/core/memGraphs/Graphs/FactGraph.h"

#include <iostream>
#include <string>

#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  FactGraph::~FactGraph()
  {
    for(auto* fact : all_facts_)
      delete fact;
    all_facts_.clear();
  }

  void FactGraph::add(ContextualizedFact* fact)
  {
    std::cout << "ADD fact " << fact->toString() << std::endl;
    all_facts_.push_back(fact);
    container_.insert(fact);
    timeline_.insert(fact->getTime(), fact);
  }

  bool FactGraph::exist(const std::string& fact_id)
  {
    return (find(fact_id) != nullptr);
  }

  bool FactGraph::isActionPart(const std::string& fact_id)
  {
    auto* fact = find(fact_id);
    if(fact == nullptr)
      return false;
    else
      return fact->isPartOfAction();
  }

  std::string FactGraph::getActionPart(const std::string& fact_id)
  {
    auto* fact = find(fact_id);
    if(fact == nullptr)
      return "";
    else if(fact->isPartOfAction())
      return fact->getActionPart()->getValue();
    else
      return "";
  }

  std::string FactGraph::getData(const std::string& fact_id)
  {
    auto* fact = find(fact_id);
    if(fact == nullptr)
      return "";
    else
      return fact->getData();
  }

  SoftPoint::Ttime FactGraph::getStamp(const std::string& fact_id)
  {
    auto* fact = find(fact_id);
    if(fact == nullptr)
      return SoftPoint::default_time;
    else
      return fact->getTime();
  }

  bool FactGraph::removeFact(const std::string& fact_id)
  {
    auto* fact = container_.find(fact_id);
    if(fact == nullptr)
      return false;

    container_.erase(fact);
    return true;
  }

  ContextualizedFact* FactGraph::find(const std::string& fact_id)
  {
    return container_.find(fact_id);
  }

  ContextualizedFact* FactGraph::findRecent(const Triplet& triplet, SoftPoint::Ttime until)
  {
    for(const BplusLeaf<SoftPoint::Ttime, ContextualizedFact*>* leaf = timeline_.getLast(); leaf != nullptr; leaf = leaf->getPreviousLeaf())
    {
      for(auto* data : leaf->payload_)
      {
        if(data->getTime() < until)
          return nullptr;
        else if(data->fit(triplet))
          return data;
      }
    }
    return nullptr;
  }

} // namespace mementar
