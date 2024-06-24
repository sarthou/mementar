#ifndef MEMENTAR_OCCASIONSMANAGER_H
#define MEMENTAR_OCCASIONSMANAGER_H

#include <atomic>
#include <mutex>
#include <queue>

#include "mementar/compat/ros.h"
#include "mementar/core/Occasions/Subscription.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"
#include "ontologenius/OntologyManipulator.h"

namespace mementar {

  class OccasionsManager
  {
  public:
    explicit OccasionsManager(const std::string& name = "");
    explicit OccasionsManager(onto::OntologyManipulator* onto, const std::string& name = "");

    void run();

    void add(const Triplet& triplet);

    void stop() { run_ = false; }
    bool isRunning() const { return run_; }

  private:
    onto::OntologyManipulator* onto_;
    Subscription subscription_;
    std::atomic<bool> run_;

    compat::mem_ros::Publisher<compat::MementarOccasion> pub_;
    compat::mem_ros::Service<compat::MementarOccasionSubscription> sub_service_;
    compat::mem_ros::Service<compat::MementarOccasionUnsubscription> unsub_service_;

    std::mutex mutex_;

    bool queue_choice_;
    std::queue<Triplet> fifo_1_;
    std::queue<Triplet> fifo_2_;

    bool subscribeCallback(compat::mem_ros::ServiceWrapper<compat::MementarOccasionSubscription::Request>& req,
                           compat::mem_ros::ServiceWrapper<compat::MementarOccasionSubscription::Response>& res);

    bool unsubscribeCallback(compat::mem_ros::ServiceWrapper<compat::MementarOccasionUnsubscription::Request>& req,
                             compat::mem_ros::ServiceWrapper<compat::MementarOccasionUnsubscription::Response>& res);

    Triplet get();
    bool empty();
  };

} // namespace mementar

#endif // MEMENTAR_OCCASIONSMANAGER_H
