#include "mementar/core/feeder/Feeder.h"

#include <iostream>

#include "mementar/core/memGraphs/Timeline.h"

namespace mementar {

Feeder::Feeder(Timeline* timeline) : callback_([this](auto triplet){ this->defaultCallback(triplet); }) //, versionor_(&feed_storage_)
{
  timeline_ = timeline;
  onto_ = nullptr;
}

Feeder::Feeder(OntologyManipulator* onto, Timeline* timeline) : callback_([this](auto triplet){ this->defaultCallback(triplet); }) //, versionor_(&feed_storage_)
{
  timeline_ = timeline;
  onto_ = onto;
}

bool Feeder::run()
{
  if(timeline_ == nullptr)
    return false;

  bool has_run = runForFacts();
  has_run = has_run || runForActions();

  return has_run;
}

bool Feeder::setWhitelist(std::vector<std::string> list)
{
  if(!is_whitelist_ || (is_whitelist_.value() == true))
  {
    setList(list);
    is_whitelist_ = true;
    return true;
  }
  else
    return false;
}

bool Feeder::setBlacklist(std::vector<std::string> list)
{
  if(!is_whitelist_ || (is_whitelist_.value() == false))
  {
    setList(list);
    is_whitelist_ = false;
    return true;
  }
  else
    return false;
}

bool Feeder::runForFacts()
{
  bool has_run = false;
  std::queue<feed_fact_t> feeds = feed_storage_.getFacts();

  while(feeds.empty() == false)
  {
    has_run = true;
    feed_fact_t feed = feeds.front();
    feeds.pop();

    if((feed.cmd_ != cmd_add) && (feed.cmd_ != cmd_del))
    {
      /*if(feed.cmd_ == cmd_commit)
      {
        if(!versionor_.commit(feed.from_))
          notifications_.push_back("[FAIL][commit]" + feed.from_);
      }
      else if(feed.cmd_ == cmd_checkout)
      {
        if(!versionor_.checkout(feed.from_))
          notifications_.push_back("[FAIL][checkout]" + feed.from_);
      }*/
      continue;
    }

    /*if(!feed.checkout_)
      versionor_.insert(feed);*/

    if(feed.fact_.value().valid() == false)
    {
      notifications_.push_back("[FAIL][fact poorly formed]" + feed.fact_.value().Triplet::toString());
    }
    else
    {
      if(is_whitelist_)
      {
        if(is_whitelist_.value() == true)
        {
          if(list_.find(feed.fact_.value().predicat_) == list_.end())
            continue;
        }
        else
        {
          if(list_.find(feed.fact_.value().predicat_) != list_.end())
            continue;
        }
      }

      if(feed.expl_.size())
      {
        ContextualizedFact* explanation = nullptr;
        Fact fact_explanation = feed.expl_[0];
        for(auto& feed_expl : feed.expl_)
        {
          if(feed_expl.valid() == false)
          {
            notifications_.push_back("[FAIL][explanation poorly formed]" + feed_expl.Triplet::toString());
          }
          else
          {
            auto tmp_explanation = timeline_->facts.findRecent(feed_expl);
            if(explanation == nullptr)
            {
              explanation = tmp_explanation;
              fact_explanation = feed_expl;
            }
            else if(explanation < tmp_explanation)
            {
              explanation = tmp_explanation;
              fact_explanation = feed_expl;
            }
          }
        }

        if(explanation == nullptr)
        {
          notifications_.push_back("[FAIL][explanation does not exist]" + feed.fact_.value().Triplet::toString() + " -- " + fact_explanation.Triplet::toString());
          continue;
        }
        else
          timeline_->facts.add(new mementar::ContextualizedFact(id_generator_.get(), {feed.fact_.value(), *explanation}));
      }
      else
        timeline_->facts.add(new mementar::ContextualizedFact(id_generator_.get(), feed.fact_.value()));

      callback_(feed.fact_.value());
    }

  }

  return has_run;
}

bool Feeder::runForActions()
{
  bool has_run = false;
  std::queue<feed_action_t> feeds = feed_storage_.getActions();

  while(feeds.empty() == false)
  {
    has_run = true;
    feed_action_t feed = feeds.front();
    feeds.pop();

    /*if(!feed.checkout_)
      versionor_.insert(feed);*/

    if(feed.name_ == "")
    {
      notifications_.push_back("[FAIL][no action name]");
    }
    else if((feed.t_start_ == SoftPoint::default_time) && (feed.t_end_ == SoftPoint::default_time))
    {
      notifications_.push_back("[FAIL][no time point] action " + feed.name_);
    }
    else
    {
      auto action = timeline_->actions.find(feed.name_);
      if((feed.t_start_ != SoftPoint::default_time) && (feed.t_end_ != SoftPoint::default_time))
      {
        if(action != nullptr)
          notifications_.push_back("[FAIL][action already exists] " + feed.name_);
        else
        {
          action = new mementar::Action(feed.name_, feed.t_start_, feed.t_end_);
          timeline_->actions.add(action);
          callback_(*action->getStartFact());
          callback_(*action->getEndFact());
        }
      }
      else if(feed.t_start_ != SoftPoint::default_time)
      {
        if(action != nullptr)
          notifications_.push_back("[FAIL][action already exists] " + feed.name_);
        else
        {
          action = new mementar::Action(feed.name_, feed.t_start_);
          timeline_->actions.add(action);
          callback_(*action->getStartFact());
        }
      }
      else if(feed.t_end_ != SoftPoint::default_time)
      {
        if(action == nullptr)
          notifications_.push_back("[FAIL][action does not exist] " + feed.name_);
        else if(timeline_->actions.setEnd(feed.name_, feed.t_end_) == false)
          notifications_.push_back("[FAIL][end stamp already exist] " + feed.name_);
        else
          callback_(*action->getEndFact());
      }
    }
  }

  return has_run;
}

void Feeder::setList(const std::vector<std::string>& base_list)
{
  if(onto_ != nullptr)
  {
    for(auto& property : base_list)
    {
      auto down_properties = onto_->objectProperties.getDown(property);
      if(down_properties.size() == 0)
        down_properties = onto_->dataProperties.getDown(property);
      if(down_properties.size() != 0)
      {
        for(auto& down : down_properties)
          list_.insert(down);
      }
      else
        list_.insert(property);
    }
  }
  else
  {
    for(auto& property : base_list)
      list_.insert(property);
  }
}

} // namespace mementar
