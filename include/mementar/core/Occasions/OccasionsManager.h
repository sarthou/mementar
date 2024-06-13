#ifndef MEMENTAR_OCCASIONSMANAGER_H
#define MEMENTAR_OCCASIONSMANAGER_H

#include <mutex>
#include <queue>
#include <atomic>

#include <mementar/compat/ros.h>

#include <ontologenius/OntologyManipulator.h>

#include <mementar/core/Occasions/Subscription.h>
#include <mementar/core/memGraphs/Branchs/types/Triplet.h>

namespace mementar
{

class OccasionsManager
{
public:
  explicit OccasionsManager(std::string name = "");
  OccasionsManager(onto::OntologyManipulator* onto, std::string name = "");

  void run();

  void add(const Triplet& triplet);

  void stop() { run_ = false; }
  inline bool isRunning() { return run_; }

private:
  onto::OntologyManipulator* onto_;
  Subscription subscription_;
  std::atomic<bool> run_;

  compat::onto_ros::Publisher<compat::MementarOccasion> pub_;
  compat::onto_ros::Service<compat::MementarOccasionSubscription> sub_service_;
  compat::onto_ros::Service<compat::MementarOccasionUnsubscription> unsub_service_;

  std::mutex mutex_;

  bool queue_choice_;
  std::queue<Triplet> fifo_1;
  std::queue<Triplet> fifo_2;

  bool SubscribeCallback(compat::onto_ros::ServiceWrapper<compat::MementarOccasionSubscription::Request>& req,
                         compat::onto_ros::ServiceWrapper<compat::MementarOccasionSubscription::Response>& res);

  bool UnsubscribeCallback(compat::onto_ros::ServiceWrapper<compat::MementarOccasionUnsubscription::Request>& req,
                           compat::onto_ros::ServiceWrapper<compat::MementarOccasionUnsubscription::Response>& res);

  Triplet get();
  bool empty();
};

} // namespace mementar

#endif // MEMENTAR_OCCASIONSMANAGER_H
