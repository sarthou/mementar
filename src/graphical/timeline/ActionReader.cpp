#include "mementar/graphical/timeline/ActionReader.h"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"
#include "mementar/core/memGraphs/Branchs/types/Action.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"
#include "mementar/core/memGraphs/Graphs/FactGraph.h"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc_c.h"

namespace mementar {

  ActionReader::ActionReader() : max_level_(0),
                                 max_text_size_(0)
  {
    actions_.clear();
    actions_to_manage_.clear();
    actions_managed_.clear();
  }

  void ActionReader::read(FactGraph* graph, CvFont* font)
  {
    auto* tree = graph->getTimeline();
    auto* node = static_cast<BplusLeaf<SoftPoint::Ttime, ContextualizedFact*>*>(tree->getFirst());

    while(node != nullptr)
    {
      for(auto* fact : node->getData())
      {
        if(fact->isPartOfAction())
        {
          auto* action = fact->getActionPart();
          if(actions_.find(action->getValue()) == actions_.end())
          {
            Action_t act = getAction(action);
            actions_.insert(std::pair<std::string, Action_t>(act.name, act));
            actions_to_manage_.push_back(action);
            getTextSize(act.name, font);
          }
          else
            closeAction(action);
        }
      }

      node = node->getNextLeaf();
    }

    setLevels();
  }

  Action_t ActionReader::getAction(Action* action)
  {
    Action_t new_action(*action->getStartFact());
    new_action.name = action->getValue();
    new_action.level = 0;

    return new_action;
  }

  void ActionReader::closeAction(Action* action)
  {
    auto act_it = actions_.find(action->getValue());
    act_it->second.end = SoftPoint(*action->getEndFact());
  }

  void ActionReader::setLevels()
  {
    size_t level = 1;

    while(actions_to_manage_.empty() == false)
    {
      std::vector<Action*> unmanaged_actions;
      std::vector<Action*> actions_at_level;

      // test actions inclusion
      for(auto* action : actions_to_manage_)
      {
        if(action->isPending())
          unmanaged_actions.push_back(action);
        else
        {
          bool include_other = false;
          for(auto* action2 : actions_to_manage_)
          {
            if(action != action2)
            {
              if((action->getStartFact()->getTime() < action2->getStartFact()->getTime()) && (action->getEndFact()->getTime() > action2->getStartFact()->getTime()))
              {
                include_other = true;
                break;
              }
              else if(action2->isPending() == false)
              {
                if((action->getStartFact()->getTime() < action2->getEndFact()->getTime()) && (action->getEndFact()->getTime() > action2->getEndFact()->getTime()))
                {
                  include_other = true;
                  break;
                }
              }
            }
          }

          if(include_other == false)
          {
            actions_at_level.push_back(action);
            actions_.at(action->getName()).level = level;
            actions_managed_.push_back(action);
          }
          else
            unmanaged_actions.push_back(action);
        }
      }

      // free spaces
      actions_to_manage_ = unmanaged_actions;
      unmanaged_actions.clear();
      for(auto* action : actions_to_manage_)
      {
        bool is_candidate = true;
        for(auto* action2 : actions_at_level)
        {
          if(action2->isPending())
          {
            is_candidate = false;
            break;
          }
          if((action->getStartFact()->getTime() >= action2->getStartFact()->getTime()) && (action->isPending() || (action->getStartFact()->getTime() <= action2->getEndFact()->getTime())))
          {
            is_candidate = false;
            break;
          }
          else if(action->isPending())
          {
            if(action2->isPending() || action->getStartFact()->getTime() <= action2->getEndFact()->getTime())
            {
              is_candidate = false;
              break;
            }
          }
          else if((action->getEndFact()->getTime() >= action2->getStartFact()->getTime()) && (action->isPending() || (action->getEndFact()->getTime() <= action2->getEndFact()->getTime())))
          {
            is_candidate = false;
            break;
          }
          else if((action->getStartFact()->getTime() <= action2->getStartFact()->getTime()) && (action->isPending() || (action->getEndFact()->getTime() >= action2->getEndFact()->getTime())))
          {
            is_candidate = false;
            break;
          }
        }

        if(is_candidate == true)
        {
          actions_at_level.push_back(action);
          actions_.at(action->getName()).level = level;
          actions_managed_.push_back(action);
        }
        else
          unmanaged_actions.push_back(action);
      }

      // instantanous
      if(level == 1)
      {
        actions_to_manage_ = unmanaged_actions;
        unmanaged_actions.clear();
        for(auto* action : actions_to_manage_)
        {
          if(action->isPending() == false)
          {
            if(action->getStartFact()->getTime() == action->getEndFact()->getTime())
            {
              actions_at_level.push_back(action);
              actions_.at(action->getName()).level = level;
              actions_managed_.push_back(action);
            }
            else
              unmanaged_actions.push_back(action);
          }
          else
            unmanaged_actions.push_back(action);
        }
      }

      // set a default one
      actions_to_manage_ = unmanaged_actions;
      unmanaged_actions.clear();
      if(actions_at_level.empty())
      {
        auto* action = actions_to_manage_[0];
        actions_at_level.push_back(action);
        actions_.at(action->getName()).level = level;
        actions_managed_.push_back(action);

        for(size_t i = 1; i < actions_to_manage_.size(); i++)
          unmanaged_actions.push_back(actions_to_manage_[i]);
        actions_to_manage_ = unmanaged_actions;
      }

      level++;
    }

    max_level_ = level - 1;
  }

  void ActionReader::getTextSize(const std::string& txt, CvFont* font)
  {
    CvSize size;
    int baseline = 0;
    cvGetTextSize(txt.c_str(), font, &size, &baseline);
    size_t text_width = size.width;
    if(text_width > max_text_size_)
      max_text_size_ = text_width;
  }

} // namespace mementar
