#include "include/mementar/API/mementar/OccasionsSubscriber.h"

namespace mementar {

  OccasionsSubscriber::OccasionsSubscriber(std::function<void(const Fact&)> callback, const std::string& name, bool spin_thread)
    : sub_(name.empty() ? "mementar/occasions" : "mementar/occasions/" + name, 1000, &OccasionsSubscriber::occasionCallback, this),
      client_subscribe_(name.empty() ? "mementar/subscribe" : "mementar/subscribe/" + name),
      client_cancel_(name.empty() ? "mementar/unsubscribe" : "mementar/unsubscribe/" + name)
  {
    callback_ = callback;

    /*if (spin_thread) {
        need_to_terminate_ = false;
        spin_thread_ = std::thread(std::bind(&OccasionsSubscriber::spinThread, this));
    } else {
        spin_thread_ = {};
    }*/
  }

  OccasionsSubscriber::OccasionsSubscriber(std::function<void(const Fact&)> callback, bool spin_thread)
    : OccasionsSubscriber(callback, "", spin_thread) {}

  OccasionsSubscriber::~OccasionsSubscriber()
  {
    cancel();
    // todo
    // sub_.shutdown();
    /*if (spin_thread_) {
        need_to_terminate_ = true;
        spin_thread_->join();
        delete spin_thread_;
    }*/
  }

  bool OccasionsSubscriber::subscribe(const Fact& pattern, size_t count)
  {
    auto req = mementar::compat::make_request<mementar::compat::MementarOccasionSubscription>();
    auto res = mementar::compat::make_response<mementar::compat::MementarOccasionSubscription>();

    [&](auto&& req) {
      req->data = pattern();
      req->count = count;
    }(mementar::compat::onto_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_subscribe_)::Status;

    if(client_subscribe_.call(req, res) != ResultTy::FAILURE)
    {
      ids_.push_back(mementar::compat::onto_ros::getServicePointer(res)->id);
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
      auto req = mementar::compat::make_request<mementar::compat::MementarOccasionUnsubscription>();
      auto res = mementar::compat::make_response<mementar::compat::MementarOccasionUnsubscription>();

      [&](auto&& req) {
        req->id = ids_[i];
      }(mementar::compat::onto_ros::getServicePointer(req));

      using ResultTy = typename decltype(client_cancel_)::Status;

      if(client_cancel_.call(req, res) != ResultTy::FAILURE)
      {
        if(mementar::compat::onto_ros::getServicePointer(res)->id != (int)ids_[i])
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

  void OccasionsSubscriber::occasionCallback(compat::MementarOccasion msg)
  {
    auto it = std::find(ids_.begin(), ids_.end(), msg.id);
    if(it != ids_.end())
    {
      callback_(Fact(msg.data));
      if(msg.last == true)
        ids_.erase(it);
    }
  }

  void OccasionsSubscriber::spinThread()
  {
    while(compat::onto_ros::Node::ok())
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
