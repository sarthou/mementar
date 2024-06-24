#include "include/mementar/API/mementar/OccasionsSubscriber.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <string>

#include "mementar/API/mementar/Fact.h"
#include "mementar/compat/ros.h"

namespace mementar {

  OccasionsSubscriber::OccasionsSubscriber(const std::function<void(const Fact&)>& callback, const std::string& name)
    : sub_(name.empty() ? "mementar/occasions" : "mementar/occasions/" + name, 1000, &OccasionsSubscriber::occasionCallback, this),
      client_subscribe_(name.empty() ? "mementar/subscribe" : "mementar/subscribe/" + name),
      client_cancel_(name.empty() ? "mementar/unsubscribe" : "mementar/unsubscribe/" + name),
      callback_(callback)
  {}

  OccasionsSubscriber::~OccasionsSubscriber()
  {
    cancel();
  }

  bool OccasionsSubscriber::subscribe(const Fact& pattern, size_t count)
  {
    auto req = mementar::compat::makeRequest<mementar::compat::MementarOccasionSubscription>();
    auto res = mementar::compat::makeResponse<mementar::compat::MementarOccasionSubscription>();

    [&](auto&& req) {
      req->data = pattern();
      req->count = count;
    }(mementar::compat::mem_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_subscribe_)::RosStatus_e;

    if(client_subscribe_.call(req, res) != ResultTy::ros_status_failure)
    {
      ids_.push_back(mementar::compat::mem_ros::getServicePointer(res)->id);
      return true;
    }
    else
      return false;
  }

  bool OccasionsSubscriber::cancel()
  {
    bool done = true;
    for(size_t i = 0; i < ids_.size();)
    {
      auto req = mementar::compat::makeRequest<mementar::compat::MementarOccasionUnsubscription>();
      auto res = mementar::compat::makeResponse<mementar::compat::MementarOccasionUnsubscription>();

      [&](auto&& req) {
        req->id = ids_[i];
      }(mementar::compat::mem_ros::getServicePointer(req));

      using ResultTy = typename decltype(client_cancel_)::RosStatus_e;

      if(client_cancel_.call(req, res) != ResultTy::ros_status_failure)
      {
        if(mementar::compat::mem_ros::getServicePointer(res)->id != (int)ids_[i])
        {
          done = false;
        }
      }
      else
      {
        done = false;
      }

      if(done)
        ids_.erase(ids_.begin() + i);
      else
        i++;
    }

    return done;
  }

  void OccasionsSubscriber::occasionCallback(const compat::MementarOccasion& msg)
  {
    auto it = std::find(ids_.begin(), ids_.end(), msg.id);
    if(it != ids_.end())
    {
      callback_(Fact(msg.data));
      if(msg.last != 0)
        ids_.erase(it);
    }
  }

  void OccasionsSubscriber::spinThread()
  {
    while(compat::mem_ros::Node::ok())
    {
      if(need_to_terminate_)
      {
        break;
      }
      // todo
      // callback_queue_.callAvailable(ros::WallDuration(0.1));
    }
  }

} // namespace mementar
