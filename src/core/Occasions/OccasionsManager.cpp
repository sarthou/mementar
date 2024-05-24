#include "mementar/core/Occasions/OccasionsManager.h"

namespace mementar {
    OccasionsManager::OccasionsManager(std::string name)
        : onto_(nullptr),
          run_(false),
          pub_((name == "") ? "occasions" : "occasions/" + name, 1000),
          sub_service_(
              name.empty() ? "subscribe" : "subscribe/" + name,
              &OccasionsManager::SubscribeCallback, this),
          unsub_service_(
              name.empty() ? "unsubscribe" : "unsubscribe/" + name,
              &OccasionsManager::UnsubscribeCallback, this) {
    }

    OccasionsManager::OccasionsManager(onto::OntologyManipulator* onto, std::string name)
        : onto_(onto),
          subscription_(onto),
          run_(false),
          pub_((name == "") ? "occasions" : "occasions/" + name, 1000),
          sub_service_(name.empty() ? "subscribe" : "subscribe/" + name, &OccasionsManager::SubscribeCallback, this),
          unsub_service_(name.empty() ? "unsubscribe" : "unsubscribe/" + name, &OccasionsManager::UnsubscribeCallback,
                         this) {
    }

    void OccasionsManager::run() {
        run_ = true;
        compat::onto_ros::Rate r(50);

        while (compat::onto_ros::Node::ok() && isRunning()) {
            while (!empty()) {
                Triplet triplet = get();
                if (triplet.valid()) {
                    std::vector<size_t> ids = subscription_.evaluate(triplet);
                    for (const auto id: ids) {
                        compat::MementarOccasion msg;
                        msg.id = id;
                        msg.data = triplet.toString();
                        msg.last = subscription_.isFinished(id);
                        pub_.publish(msg);
                    }
                }
            }
            r.sleep();
        }
    }

    void OccasionsManager::add(const Triplet& triplet) {
        mutex_.lock();
        if (queue_choice_ == true)
            fifo_1.push(triplet);
        else
            fifo_2.push(triplet);
        mutex_.unlock();
    }

    bool OccasionsManager::SubscribeCallback(
        compat::onto_ros::ServiceWrapper<compat::MementarOccasionSubscription::Request>& req,
        compat::onto_ros::ServiceWrapper<compat::MementarOccasionSubscription::Response>& res
    ) {
        return [this](auto&& req, auto&& res) {
            auto triplet_patern = TripletPattern::deserialize(req->data);

            if (!triplet_patern.valid())
                return false;

            res->id = subscription_.subscribe(triplet_patern, req->count);

            return true;
        }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
    }

    bool OccasionsManager::UnsubscribeCallback(
        compat::onto_ros::ServiceWrapper<compat::MementarOccasionUnsubscription::Request>& req,
        compat::onto_ros::ServiceWrapper<compat::MementarOccasionUnsubscription::Response>& res
    ) {
        return [this](auto&& req, auto&& res) {
            if (subscription_.unsubscribe(req->id))
                res->id = req->id;
            else
                res->id = -1;

            return true;
        }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
    }

    Triplet OccasionsManager::get() {
        Triplet res;
        mutex_.lock();
        if (queue_choice_ == true) {
            if (!fifo_2.empty()) {
                res = fifo_2.front();
                fifo_2.pop();
            }

            if (fifo_2.empty())
                queue_choice_ = false;
        } else {
            if (!fifo_1.empty()) {
                res = fifo_1.front();
                fifo_1.pop();
            }

            if (fifo_1.empty())
                queue_choice_ = true;
        }
        mutex_.unlock();
        return res;
    }

    bool OccasionsManager::empty() {
        bool res = true;
        mutex_.lock();
        res = fifo_2.empty() && fifo_1.empty();
        mutex_.unlock();
        return res;
    }
} // namespace mementar
