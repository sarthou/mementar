#ifndef MEMENTAR_FEEDER_H
#define MEMENTAR_FEEDER_H

#include "mementar/core/feeder/FeedStorage.h"
#include "mementar/core/feeder/IdGenerator.h"
#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"
// #include "ontologenius/core/feeder/Versionor.h"

#include <functional>
#include <unordered_set>

#include "ontologenius/OntologyManipulator.h"

namespace mementar {

  class Timeline;

  class Feeder
  {
  public:
    explicit Feeder(onto::OntologyManipulator* onto, Timeline* timeline = nullptr);
    Feeder(Timeline* timeline = nullptr);

    void storeFact(const std::string& feed, const SoftPoint::Ttime& stamp) { feed_storage_.insertFact(feed, stamp); }
    void storeFact(const std::string& feed, const std::string& expl) { feed_storage_.insertFact(feed, expl); }
    void storeAction(const std::string& name, const SoftPoint::Ttime& start_stamp, const SoftPoint::Ttime& end_stamp) { feed_storage_.insertAction(name, start_stamp, end_stamp); }
    bool run();
    void link(Timeline* timeline)
    {
      timeline_ = timeline;
      id_generator_.reset();
    }
    void setCallback(const std::function<void(ContextualizedFact*)>& callback) { callback_ = callback; }

    bool setWhitelist(const std::vector<std::string>& list);
    bool setBlacklist(const std::vector<std::string>& list);

    std::vector<std::string> getNotifications()
    {
      auto tmp = std::move(notifications_);
      notifications_.clear();
      return tmp;
    }

    // void activateVersionning(bool activated) { versionor_.activate(activated); }
    // void exportToXml(const std::string& path) { versionor_.exportToXml(path); }

    size_t size() { return feed_storage_.size(); }

  private:
    FeedStorage feed_storage_;
    IdGenerator id_generator_;
    // Versionor versionor_;
    Timeline* timeline_;
    onto::OntologyManipulator* onto_;
    std::function<void(ContextualizedFact*)> callback_;

    std::optional<bool> is_whitelist_;
    std::unordered_set<std::string> list_;

    // Here the notifications are about miss formed queries
    std::vector<std::string> notifications_;

    bool runForFacts();
    bool runForActions();

    void setList(const std::vector<std::string>& base_list);

    void defaultCallback(ContextualizedFact* fact) {}
  };

} // namespace mementar

#endif // MEMENTAR_FEEDER_H
