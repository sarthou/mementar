#include "mementar/core/memGraphs/Graphs/ActionGraph.h"

#include <algorithm>
#include <iterator>
#include <string>
#include <unordered_set>
#include <vector>

#include "mementar/core/memGraphs/Branchs/types/Action.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Graphs/FactGraph.h"

namespace mementar {
  ActionGraph::ActionGraph(FactGraph* fact_graph) : fact_graph_(fact_graph)
  {
  }

  ActionGraph::~ActionGraph()
  {
    for(auto* action : all_actions_)
      delete action;
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
    for(const auto& act : pending_actions_)
      res.insert(act.second->getName());
    return res;
  }

  bool ActionGraph::isPending(const std::string& action_name)
  {
    auto* action_branch = find(action_name);
    if(action_branch == nullptr)
      return false;
    else
      return action_branch->isPending();
  }

  SoftPoint::Ttime ActionGraph::getStartStamp(const std::string& action_name)
  {
    auto* action_branch = find(action_name);
    if(action_branch == nullptr)
      return SoftPoint::default_time;
    else
      return action_branch->getStartFact()->getTime();
  }

  SoftPoint::Ttime ActionGraph::getEndStamp(const std::string& action_name)
  {
    auto* action_branch = find(action_name);
    if(action_branch == nullptr)
      return SoftPoint::default_time;
    else if(action_branch->isPending())
      return SoftPoint::default_time;
    else
      return action_branch->getEndFact()->getTime();
  }

  SoftPoint::Ttime ActionGraph::getDuration(const std::string& action_name)
  {
    auto* action_branch = find(action_name);
    if(action_branch == nullptr)
      return SoftPoint::default_time;
    else
      return action_branch->getDuration();
  }

  std::string ActionGraph::getStartFact(const std::string& action_name)
  {
    auto* action_branch = find(action_name);
    if(action_branch == nullptr)
      return "";
    else
      return action_branch->getStartFact()->getValue();
  }

  std::string ActionGraph::getEndFact(const std::string& action_name)
  {
    auto* action_branch = find(action_name);
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
    auto* action_branch = find(action_name);
    if(action_branch != nullptr)
    {
      auto* start_fact = action_branch->getStartFact();
      auto* leaf = start_fact->getLeaf()->getNextLeaf();
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
        auto* end_fact = action_branch->getEndFact();
        while(leaf != nullptr)
        {
          auto data = leaf->getData();
          if(data.empty() == false)
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

  bool ActionGraph::removeAction(const std::string& action_name)
  {
    auto* action = container_.find(action_name);
    if(action == nullptr)
      return false;

    auto* start_fact = action->getStartFact();
    if(start_fact != nullptr)
    {
      if(fact_graph_->removeFact(start_fact->getId()) == false)
        return false;
    }

    if(action->isPending() == false)
    {
      auto* end_fact = action->getEndFact();
      if(end_fact != nullptr)
      {
        if(fact_graph_->removeFact(end_fact->getId()) == false)
          return false;
      }
    }
    else
      pending_actions_.erase(action->getValue());

    auto action_it = std::find(all_actions_.begin(), all_actions_.end(), action);
    if(action_it != all_actions_.end())
      all_actions_.erase(action_it);

    container_.erase(action);
    delete action;

    return true;
  }

  std::vector<Action*> ActionGraph::getPendingPtr()
  {
    std::vector<Action*> res;
    std::transform(pending_actions_.cbegin(), pending_actions_.cend(), std::back_inserter(res),
                   [](const auto& action) { return action.second; });
    return res;
  }

  Action* ActionGraph::find(const std::string& action_name)
  {
    return container_.find(action_name);
  }

} // namespace mementar
