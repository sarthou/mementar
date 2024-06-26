#include "mementar/core/Occasions/OccasionsManager.h"

#include <cstddef>
#include <string>
#include <vector>

#include "mementar/compat/ros.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"
#include "mementar/core/memGraphs/Branchs/types/TripletPattern.h"
#include "ontologenius/OntologyManipulator.h"

namespace mementar {

  OccasionsManager::OccasionsManager(const std::string& name)
    : run_(false),
      pub_((name.empty()) ? "mementar/occasions" : "mementar/occasions/" + name, 1000),
      sub_service_(
        name.empty() ? "mementar/subscribe" : "mementar/subscribe/" + name,
        &OccasionsManager::subscribeCallback, this),
      unsub_service_(
        name.empty() ? "mementar/unsubscribe" : "mementar/unsubscribe/" + name,
        &OccasionsManager::unsubscribeCallback, this),
      queue_choice_(true)
  {
  }

  OccasionsManager::OccasionsManager(onto::OntologyManipulator* onto, const std::string& name)
    : subscription_(onto),
      run_(false),
      pub_((name.empty()) ? "mementar/occasions" : "mementar/occasions/" + name, 1000),
      sub_service_(name.empty() ? "mementar/subscribe" : "mementar/subscribe/" + name, &OccasionsManager::subscribeCallback, this),
      unsub_service_(name.empty() ? "mementar/unsubscribe" : "mementar/unsubscribe/" + name, &OccasionsManager::unsubscribeCallback,
                     this),
      queue_choice_(true)
  {
  }

  void OccasionsManager::run()
  {
    run_ = true;
    compat::mem_ros::Rate r(50);

    while(compat::mem_ros::Node::ok() && isRunning())
    {
      while(!empty())
      {
        Triplet triplet = get();
        if(triplet.valid())
        {
          std::vector<size_t> ids = subscription_.evaluate(triplet);
          for(auto id : ids)
          {
            compat::MementarOccasion msg;
            msg.id = (int)id;
            msg.data = triplet.toString();
            msg.last = subscription_.isFinished(id);
            if(msg.last)
              subscription_.unsubscribe(id);
            pub_.publish(msg);
          }
        }
      }
      r.sleep();
    }
  }

  void OccasionsManager::add(const Triplet& triplet)
  {
    mutex_.lock();
    if(queue_choice_ == true)
      fifo_1_.push(triplet);
    else
      fifo_2_.push(triplet);
    mutex_.unlock();
  }

  bool OccasionsManager::subscribeCallback(compat::mem_ros::ServiceWrapper<compat::MementarOccasionSubscription::Request>& req,
                                           compat::mem_ros::ServiceWrapper<compat::MementarOccasionSubscription::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      auto triplet_patern = TripletPattern::deserialize(req->data);

      if(!triplet_patern.valid())
        return false;

      res->id = subscription_.subscribe(triplet_patern, req->count);

      return true;
    }(compat::mem_ros::getServicePointer(req), compat::mem_ros::getServicePointer(res));
  }

  bool OccasionsManager::unsubscribeCallback(compat::mem_ros::ServiceWrapper<compat::MementarOccasionUnsubscription::Request>& req,
                                             compat::mem_ros::ServiceWrapper<compat::MementarOccasionUnsubscription::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      if(subscription_.unsubscribe(req->id))
        res->id = req->id;
      else
        res->id = -1;

      return true;
    }(compat::mem_ros::getServicePointer(req), compat::mem_ros::getServicePointer(res));
  }

  Triplet OccasionsManager::get()
  {
    Triplet res;
    mutex_.lock();
    if(queue_choice_ == true)
    {
      if(!fifo_2_.empty())
      {
        res = fifo_2_.front();
        fifo_2_.pop();
      }

      if(fifo_2_.empty())
        queue_choice_ = false;
    }
    else
    {
      if(!fifo_1_.empty())
      {
        res = fifo_1_.front();
        fifo_1_.pop();
      }

      if(fifo_1_.empty())
        queue_choice_ = true;
    }
    mutex_.unlock();
    return res;
  }

  bool OccasionsManager::empty()
  {
    bool res = true;
    mutex_.lock();
    res = fifo_2_.empty() && fifo_1_.empty();
    mutex_.unlock();
    return res;
  }
} // namespace mementar
