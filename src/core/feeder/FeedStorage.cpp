#include "mementar/core/feeder/FeedStorage.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <iostream>
#include <queue>
#include <regex>
#include <string>
#include <vector>

#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

namespace mementar {

  FeedStorage::FeedStorage() : base_regex_(R"(^\[(\w+)\](.*)$)")
  {}

  void FeedStorage::insertFact(const std::string& regex, const SoftPoint::Ttime& stamp)
  {
    FeedFact_t feed = getFeedFact(regex, stamp);
    fact_mutex_.lock();
    fact_queue_.push(feed);
    fact_mutex_.unlock();
  }

  void FeedStorage::insertFact(const std::string& regex, const std::string& expl_regex)
  {
    FeedFact_t feed = getFeedFact(regex);
    size_t expl_pose = expl_regex.find(']') + 1;
    std::string expl_action = expl_regex.substr(0, expl_pose);

    auto explanations = split(expl_regex.substr(expl_pose), ";");
    for(auto& expl : explanations)
      feed.expl_.push_back(getFeedFact(expl_action + expl).fact_.value());
    fact_mutex_.lock();
    fact_queue_.push(feed);
    fact_mutex_.unlock();
  }

  void FeedStorage::insertFacts(std::vector<FeedFact_t>& datas)
  {
    fact_mutex_.lock();
    for(auto& data : datas)
      fact_queue_.push(data);
    fact_mutex_.unlock();
  }

  void FeedStorage::insertAction(const std::string& name,
                                 const SoftPoint::Ttime& start_stamp,
                                 const SoftPoint::Ttime& end_stamp)
  {
    FeedAction_t feed;
    feed.name_ = name;
    feed.t_start_ = start_stamp;
    feed.t_end_ = end_stamp;
    action_mutex_.lock();
    action_queue_.push(feed);
    action_mutex_.unlock();
  }

  void FeedStorage::insertActions(std::vector<FeedAction_t>& datas)
  {
    action_mutex_.lock();
    for(auto& data : datas)
      action_queue_.push(data);
    action_mutex_.unlock();
  }

  std::queue<FeedFact_t> FeedStorage::getFacts()
  {
    fact_mutex_.lock();
    std::queue<FeedFact_t> tmp;
    fact_queue_.swap(tmp);
    fact_mutex_.unlock();
    return tmp;
  }

  std::queue<FeedAction_t> FeedStorage::getActions()
  {
    action_mutex_.lock();
    std::queue<FeedAction_t> tmp;
    action_queue_.swap(tmp);
    action_mutex_.unlock();
    return tmp;
  }

  FeedFact_t FeedStorage::getFeedFact(const std::string& regex, const SoftPoint::Ttime& stamp)
  {
    std::smatch base_match;
    FeedFact_t feed;
    feed.cmd_ = cmd_nop;

    if(std::regex_match(regex, base_match, base_regex_))
    {
      if(base_match.size() == 3)
      {
        std::string action = base_match[1].str();
        std::transform(action.begin(), action.end(), action.begin(), ::tolower);
        if(action == "add")
          feed.cmd_ = cmd_add;
        else if(action == "del")
          feed.cmd_ = cmd_del;
        else if(action == "nop")
          return feed;
        else
        {
          std::cout << "data do not match" << std::endl;
          return feed;
        }
      }
    }
    else if(regex == "[nop]")
      return feed;
    else
    {
      std::cout << "data do not match" << std::endl;
      return feed;
    }

    feed.fact_ = Fact(Triplet(base_match[2].str(), feed.cmd_ == cmd_add), stamp);
    return feed;
  }

  std::vector<std::string> FeedStorage::split(const std::string& str, const std::string& delim)
  {
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
      pos = str.find(delim, prev);
      if(pos == std::string::npos)
        pos = str.length();

      std::string token = str.substr(prev, pos - prev);

      if(!token.empty())
        tokens.push_back(token);
      prev = pos + delim.length();
    } while((pos < str.length()) && (prev < str.length()));

    return tokens;
  }

} // namespace mementar
