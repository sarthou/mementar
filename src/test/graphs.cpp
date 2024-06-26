#include <iostream>
#include <string>

#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"
#include "mementar/core/memGraphs/Branchs/types/Action.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Graphs/ActionGraph.h"
#include "mementar/core/memGraphs/Graphs/FactGraph.h"

void printNext(mementar::ContextualizedFact* fact)
{
  auto nexts = fact->getNextData();
  for(auto* n : nexts)
    std::cout << n->toString() << std::endl;

  if(nexts.empty() == false)
    printNext(nexts[0]);
}

void print(mementar::ContextualizedFact* fact)
{
  std::cout << fact->toString() << std::endl;

  auto nexts = fact->getNextData();
  for(auto* n : nexts)
    std::cout << n->toString() << std::endl;

  if(nexts.empty() == false)
    printNext(nexts[0]);
}

void printEventList(mementar::ContextualizedFact* fact)
{
  if(fact != nullptr)
  {
    std::cout << fact->toString() << std::endl;
    printEventList(fact->getNext());
  }
}

int main()
{
  mementar::FactGraph fact_graph;
  mementar::ActionGraph action_graph(&fact_graph);

  mementar::Fact e1("cube12|isOn|Table_1", 2, 4);
  mementar::Fact e2("cube12|isOn|Table_2", 8);

  action_graph.add(new mementar::Action("pick_1", mementar::SoftPoint(1, 3), mementar::SoftPoint(7)));
  action_graph.add(new mementar::Action("place_1", mementar::SoftPoint(7), mementar::SoftPoint(9, 10)));
  action_graph.add(new mementar::Action("pick_2", mementar::SoftPoint(12)));

  auto pending = action_graph.getPendingPtrSafe();
  for(auto* action : pending)
    std::cout << action->getName() << " is a pending action" << std::endl;

  action_graph.setEnd("pick_2", mementar::SoftPoint(15));
  action_graph.setEnd("pick_1", mementar::SoftPoint(17));

  fact_graph.add(new mementar::ContextualizedFact("ce3", e1));
  fact_graph.add(new mementar::ContextualizedFact("ce4", e2));

  for(mementar::SoftPoint::Ttime i = 10; i < 20; i++)
    fact_graph.add(new mementar::ContextualizedFact("ce" + std::to_string(i), mementar::Fact("cube" + std::to_string(i) + "|isOn|Table_3", i)));

  auto actions = action_graph.getSafe();
  for(auto* action : actions)
    std::cout << action->getName() << " D=" << action->getDuration() << " Dmin=" << action->getMinDuration() << " Dmax=" << action->getMaxDuration() << std::endl;

  auto events = fact_graph.getSafe();
  for(auto* event : events)
    std::cout << event->toString() << std::endl;

  auto* timeline = fact_graph.getTimeline();
  timeline->displayTree();

  std::cout << "********" << std::endl;
  auto* first_evt = fact_graph.findBranch("pick_1_start");
  print(first_evt);

  std::cout << "********" << std::endl;
  auto* cube12_ison_evt = fact_graph.findBranch("ce3");
  printEventList(cube12_ison_evt);

  return 0;
}
