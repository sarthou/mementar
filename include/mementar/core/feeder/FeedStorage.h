#ifndef MEMENTER_FEEDSTORAGE_H
#define MEMENTER_FEEDSTORAGE_H

#include <mutex>
#include <queue>
#include <regex>
#include <string>
#include <vector>

#include "mementar/core/memGraphs/Branchs/types/Fact.h"

namespace mementar {

enum feed_commande_t
{
  cmd_add,
  cmd_del,
  cmd_commit,
  cmd_checkout,
  cmd_nop
};

struct feed_fact_t
{
  feed_commande_t cmd_;
  std::experimental::optional<Fact> fact_;
  std::vector<Fact> expl_;
  bool checkout_;

  feed_fact_t() : cmd_(cmd_nop),
                  checkout_(false) {}
};

struct feed_action_t
{
  std::string name_;
  SoftPoint::Ttime t_start_;
  SoftPoint::Ttime t_end_;
  bool checkout_;

  feed_action_t() : t_start_(SoftPoint::default_time),
                    t_end_(SoftPoint::default_time),
                    checkout_(false)
  {}
};

class FeedStorage
{
public:
  FeedStorage();

  void insertFact(const std::string& regex, const SoftPoint::Ttime& stamp);
  void insertFact(const std::string& regex, const std::string& expl_regex);
  void insertFacts(std::vector<feed_fact_t>& datas);
  void insertAction(const std::string& name, const SoftPoint::Ttime& start_stamp, const SoftPoint::Ttime& end_stamp);
  void insertActions(std::vector<feed_action_t>& datas);

  std::queue<feed_fact_t> getFacts();
  std::queue<feed_action_t> getActions();

  size_t size()
  {
    fact_mutex_.lock();
    action_mutex_.lock();
    std::cout << fact_queue_.size() << " : " << action_queue_.size() << std::endl;
    size_t queues_size = fact_queue_.size() + action_queue_.size();
    action_mutex_.unlock();
    fact_mutex_.unlock();
    return queues_size;
  }

private:
  std::regex base_regex;

  std::mutex fact_mutex_;
  std::mutex action_mutex_;
  std::queue<feed_fact_t> fact_queue_;
  std::queue<feed_action_t> action_queue_;

  feed_fact_t getFeedFact(const std::string& regex, const SoftPoint::Ttime& stamp = SoftPoint::default_time);
  std::vector<std::string> split(const std::string& str, const std::string& delim);
};

} // namespace mementar

#endif // MEMENTER_FEEDSTORAGE_H
