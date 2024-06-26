#ifndef MEMENTAR_ACTIONGRAPH_H
#define MEMENTAR_ACTIONGRAPH_H

#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"
#include "mementar/core/memGraphs/Branchs/types/Action.h"
#include "mementar/core/memGraphs/Graphs/FactGraph.h"
#include "mementar/core/memGraphs/Graphs/Graph.h"

namespace mementar {

  class ActionGraph : public Graph<Action>
  {
  public:
    explicit ActionGraph(FactGraph* fact_graph);
    ~ActionGraph() override;

    void add(Action* action);
    bool setEnd(const std::string& action_name, const SoftPoint& end);

    bool exist(const std::string& action_name);
    std::unordered_set<std::string> getPending();
    bool isPending(const std::string& action_name);
    SoftPoint::Ttime getStartStamp(const std::string& action_name);
    SoftPoint::Ttime getEndStamp(const std::string& action_name);
    SoftPoint::Ttime getDuration(const std::string& action_name);
    std::string getStartFact(const std::string& action_name);
    std::string getEndFact(const std::string& action_name);
    std::unordered_set<std::string> getFactsDuring(const std::string& action_name);
    bool removeAction(const std::string& action_name);

    std::vector<Action*> get() override
    {
      return all_actions_;
    }

    std::vector<Action*> getSafe() override
    {
      std::shared_lock<std::shared_timed_mutex> lock(Graph<Action>::mutex_);
      return all_actions_;
    }

    std::vector<Action*> getPendingPtr();
    std::vector<Action*> getPendingPtrSafe()
    {
      std::shared_lock<std::shared_timed_mutex> lock(Graph<Action>::mutex_);
      return getPendingPtr();
    }

    Action* find(const std::string& action_name);

  private:
    FactGraph* fact_graph_;
    std::vector<Action*> all_actions_;
    std::map<std::string, Action*> pending_actions_;
  };

} // namespace mementar

#endif // MEMENTAR_ACTIONGRAPH_H
