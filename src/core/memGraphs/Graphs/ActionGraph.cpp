#include "mementar/core/memGraphs/Graphs/ActionGraph.h"

namespace mementar {

  ActionGraph::ActionGraph(FactGraph* fact_graph)
  {
    fact_graph_ = fact_graph;
  }

  ActionGraph::~ActionGraph()
  {
    for(size_t i = 0; i < all_actions_.size(); i++)
      delete all_actions_[i];
    all_actions_.clear();
  }

  void ActionGraph::add(Action* action)
  {
    all_actions_.push_back(action);
    container_.insert(action);
    fact_graph_->add(action->getStartFact());
    if(action->isPending())
      pending_actions_[action->getName()] = action;
    else
      fact_graph_->add(action->getEndFact());
  }

  bool ActionGraph::setEnd(const std::string& action_name, const SoftPoint& end)
  {
    auto action_it = pending_actions_.find(action_name);
    if(action_it == pending_actions_.end())
      return false;
    else
    {
      if(action_it->second->setEnd(end))
      {
        fact_graph_->add(action_it->second->getEndFact());
        pending_actions_.erase(action_it);
        return true;
      }
      else
        return false;
    }
  }

  bool ActionGraph::exist(const std::string& action_name)
  {
    return (find(action_name) != nullptr);
  }

  std::unordered_set<std::string> ActionGraph::getPending()
  {
    std::unordered_set<std::string> res;
    for(auto act : pending_actions_)
      res.insert(act.second->getName());
    return res;
  }

  bool ActionGraph::isPending(const std::string& action_name)
  {
    auto action_branch = find(action_name);
    if(action_branch == nullptr)
      return false;
    else
      return action_branch->isPending();
  }

  SoftPoint::Ttime ActionGraph::getStartStamp(const std::string& action_name)
  {
    auto action_branch = find(action_name);
    if(action_branch == nullptr)
      return SoftPoint::default_time;
    else
      return action_branch->getStartFact()->getTime();
  }

  SoftPoint::Ttime ActionGraph::getEndStamp(const std::string& action_name)
  {
    auto action_branch = find(action_name);
    if(action_branch == nullptr)
      return SoftPoint::default_time;
    else if(action_branch->isPending())
      return SoftPoint::default_time;
    else
      return action_branch->getEndFact()->getTime();
  }

  SoftPoint::Ttime ActionGraph::getDuration(const std::string& action_name)
  {
    auto action_branch = find(action_name);
    if(action_branch == nullptr)
      return SoftPoint::default_time;
    else
      return action_branch->getDuration();
  }

  std::string ActionGraph::getStartFact(const std::string& action_name)
  {
    auto action_branch = find(action_name);
    if(action_branch == nullptr)
      return "";
    else
      return action_branch->getStartFact()->getValue();
  }

  std::string ActionGraph::getEndFact(const std::string& action_name)
  {
    auto action_branch = find(action_name);
    if(action_branch == nullptr)
      return "";
    else if(action_branch->isPending())
      return "";
    else
      return action_branch->getEndFact()->getValue();
  }

  std::unordered_set<std::string> ActionGraph::getFactsDuring(const std::string& action_name)
  {
    std::unordered_set<std::string> res;
    auto action_branch = find(action_name);
    if(action_branch != nullptr)
    {
      auto start_fact = action_branch->getStartFact();
      auto leaf = start_fact->getLeaf()->getNextLeaf();
      if(action_branch->isPending())
      {
        while(leaf != nullptr)
        {
          auto data = leaf->getData();
          for(auto& fact : data)
            res.insert(fact->getValue());
          leaf = leaf->getNextLeaf();
        }
      }
      else
      {
        auto end_fact = action_branch->getEndFact();
        while(leaf != nullptr)
        {
          auto data = leaf->getData();
          if(data.size())
            if(data[0]->getTime() >= end_fact->getTime())
              break;

          for(auto& fact : data)
            res.insert(fact->getValue());
          leaf = leaf->getNextLeaf();
        }
      }
    }

    return res;
  }

  std::vector<Action*> ActionGraph::getPendingPtr()
  {
    std::vector<Action*> res;
    for(auto act : pending_actions_)
      res.push_back(act.second);
    return res;
  }

  Action* ActionGraph::find(const std::string& action_name)
  {
    return container_.find(action_name);
  }

} // namespace mementar
