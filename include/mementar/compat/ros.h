#ifndef COMPAT_MEME_ROS_H
#define COMPAT_MEME_ROS_H

#if MEME_ROS_VERSION == 1
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

// Commonly used built-in interfaces
#include <std_msgs/String.h>

// User-defined message interfaces
#include <mementar/MementarAction.h>
#include <mementar/MementarExplanation.h>
#include <mementar/MementarOccasion.h>
#include <mementar/MementarTimestamp.h>
#include <mementar/StampedFact.h>
#include <mementar/StampedString.h>

// User-defined service interfaces
#include <mementar/MementarOccasionUnsubscription.h>
#include <mementar/MementarOccasionSubscription.h>
#include <mementar/MementarService.h>

namespace std_msgs_compat = std_msgs;

#elif MEME_ROS_VERSION == 2

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Commonly used built-in interfaces
#include <std_msgs/msg/string.hpp>

// User-defined message interfaces
#include <mementar/msg/mementar_action.hpp>
#include <mementar/msg/mementar_explanation.hpp>
#include <mementar/msg/mementar_occasion.hpp>
#include <mementar/msg/mementar_timestamp.hpp>
#include <mementar/msg/stamped_fact.hpp>
#include <mementar/msg/stamped_string.hpp>

// User-defined service interfaces
#include <mementar/srv/mementar_occasion_unsubscription.hpp>
#include <mementar/srv/mementar_occasion_subscription.hpp>
#include <mementar/srv/mementar_service.hpp>

#include <tuple>

namespace std_msgs_compat = std_msgs::msg;

#else
#error The macro MEME_ROS_VERSION is not defined, please check your build system.
#endif

#include <string>
#include <memory>
#include <functional>
#include <map>
#include <mutex>

namespace mementar::compat {
#if MEME_ROS_VERSION == 1
using namespace ::mementar;

template <typename T>
using RawRequestType = typename T::Request;

template <typename T>
using RawResponseType = typename T::Response;

template <typename T>
using RequestType = typename T::Request;

template <typename T>
using ResponseType = typename T::Response;

template <typename T, typename Request_ = typename T::Request>
inline auto make_request() { return Request_(); }

template <typename T, typename Response_ = typename T::Response>
inline auto make_response() { return Response_(); }

// todo: RequestType, ResponseType

#elif MEME_ROS_VERSION == 2
using namespace ::mementar::msg;
using namespace ::mementar::srv;

template<typename T>
using RawRequestType = typename T::Request;

template<typename T>
using RawResponseType = typename T::Response;

template<typename T>
using RequestType = std::shared_ptr<typename T::Request>;

template<typename T>
using ResponseType = std::shared_ptr<typename T::Response>;

template<typename T, typename Request_ = typename T::Request>
inline auto make_request() { return std::make_shared<Request_>(); }

template<typename T, typename Response_ = typename T::Response>
inline auto make_response() { return std::make_shared<Response_>(); }

// template <typename T, typename Result_ = typename T::>
#endif

namespace onto_ros {

#if MEME_ROS_VERSION == 1
template <typename T>
using ServiceWrapper = T;

template <typename T>
using MessageWrapper = typename T::ConstPtr;

using Rate = ros::Rate;
using RosTime = ros::Time;

template <typename T>
T* getServicePointer(T& service) { return &service; }

inline std::string getShareDirectory(const std::string& name) {
    return ros::package::getPath(name);
}

#elif MEME_ROS_VERSION == 2
template<typename T>
using ServiceWrapper = typename T::SharedPtr; //std::shared_ptr<T>;

template<typename T>
using MessageWrapper = typename T::ConstSharedPtr;

using Rate = rclcpp::Rate;
using RosTime = rclcpp::Time;

using namespace ::mementar::msg;
using namespace ::mementar::srv;

template<typename T>
T &getServicePointer(T &service) { return service; }

inline std::string getShareDirectory(const std::string &name) {
    return ament_index_cpp::get_package_share_directory(name);
}

#endif

template<typename T>
class Publisher;

template<typename T>
class Subscriber;

template<typename T>
class Service;

template<typename T>
class Client;

class Time : public RosTime {
public:
    Time(uint32_t sec, uint32_t nsec)
            : RosTime(sec, nsec) {}

    explicit Time(double t)
            : RosTime((uint32_t) t, (uint32_t) ((t - std::floor(t)) * 1'000'000'000.)) {}

    Time(const RosTime &time)
            : RosTime(time) {} // do not put it as explicit

    uint32_t seconds() const {
#if MEME_ROS_VERSION == 1
        return sec;
#elif MEME_ROS_VERSION == 2
        return RosTime::seconds();
#endif
    }

    uint32_t nanoseconds() const {
#if MEME_ROS_VERSION == 1
        return nsec;
#elif MEME_ROS_VERSION == 2
        return RosTime::nanoseconds();
#endif
    }
};

class Node {
public:
    template<typename T>
    friend
    class Publisher;

    template<typename T>
    friend
    class Subscriber;

    template<typename T>
    friend
    class Service;

    template<typename T>
    friend
    class Client;

    Node(Node &other) = delete;

    Node(Node &&other) = delete;

    ~Node() {
#if MEME_ROS_VERSION == 2
    if (ros_thread_.joinable()) ros_thread_.join();
#endif
    }

    static Node &get();

    static bool ok();

    static void init(int argc, char **argv, const std::string &node_name);

    static void shutdown();

    void spin();

    Time current_time();

private:
    explicit Node(const std::string &node_name);

    const std::string name_;

#if MEME_ROS_VERSION == 1
    ros::NodeHandle handle_;
    ros::CallbackQueue callback_queue_;
#elif MEME_ROS_VERSION == 2
    rclcpp::Node::SharedPtr handle_;
    std::thread ros_thread_;
#endif

    bool running_;
};

template<typename T>
class Publisher {
public:
    Publisher(const std::string &topic_name, std::size_t queue_size) : name_(topic_name) {
        auto &node = Node::get();

        printf("[Publisher '%s'] Create\n", topic_name.c_str());

#if MEME_ROS_VERSION == 1
        handle_ = node.handle_.advertise<T>(topic_name, queue_size);
#elif MEME_ROS_VERSION == 2
        (void) queue_size;
        handle_ = node.handle_->create_publisher<T>(topic_name, 10);
#endif
    }

    void publish(const T &message) {
        printf("[Publisher '%s'] Create\n", name_.c_str());

#if MEME_ROS_VERSION == 1
        handle_.publish(message);
#elif MEME_ROS_VERSION == 2
        handle_->publish(message);
#endif
    }

    size_t getNumSubscribers() {
#if MEME_ROS_VERSION == 1
        return handle_.getNumSubscribers();
#elif MEME_ROS_VERSION == 2
        return handle_->get_subscription_count();
#endif
    }

private:
#if MEME_ROS_VERSION == 1
    ros::Publisher handle_;
#elif MEME_ROS_VERSION == 2
    typename rclcpp::Publisher<T>::SharedPtr handle_;
#endif

    std::string name_;
};

template<typename T>
class Subscriber {
public:
    template<typename Ta, typename Tb>
    Subscriber(const std::string &topic_name, std::size_t queue_size, Ta &&callback, Tb &&ptr) {
        auto &node = Node::get();

        printf("[Subscriber '%s'] Create\n", topic_name.c_str());

#if MEME_ROS_VERSION == 1
        handle_ = node.handle_.subscribe(topic_name, queue_size, callback, ptr);
#elif MEME_ROS_VERSION == 2
        (void) queue_size;
        handle_ = node.handle_->create_subscription<T>(topic_name, 10,
                                                       std::bind(std::forward<Ta>(callback), ptr,
                                                                 std::placeholders::_1));
#endif
    }

private:
#if MEME_ROS_VERSION == 1
    ros::Subscriber handle_;
#elif MEME_ROS_VERSION == 2
    typename rclcpp::Subscription<T>::SharedPtr handle_;
#endif
};

template<typename T>
class Service {
public:
    template<typename Ta>
    Service(const std::string &service_name, Ta &&callback) {
        auto &node = Node::get();

        printf("[Service '%s'] Create\n", service_name.c_str());

#if MEME_ROS_VERSION == 1
        handle_ = node.handle_.advertiseService(service_name, callback);
#elif MEME_ROS_VERSION == 2
        handle_ = node.handle_->create_service<T>(service_name,
                                                  [&](
                                                          compat::onto_ros::ServiceWrapper<typename T::Request> req,
                                                          compat::onto_ros::ServiceWrapper<typename T::Response> res
                                                  ) {
                                                      callback(req, res);
                                                  }
        );

        //handle_ = node.handle_->create_service<T>(service_name, callback);
#endif
    }

    template<typename Ta, typename Tb>
    Service(const std::string &service_name, Ta &&callback, Tb &&ptr) {
        auto &node = Node::get();

        printf("[Service '%s'] Create\n", service_name.c_str());

#if MEME_ROS_VERSION == 1
        handle_ = node.handle_.advertiseService(service_name, callback, ptr);
#elif MEME_ROS_VERSION == 2
        handle_ = node.handle_->create_service<T>(service_name, [ptr, callback](
                compat::onto_ros::ServiceWrapper<typename T::Request> req,
                compat::onto_ros::ServiceWrapper<typename T::Response> res) { (ptr->*callback)(req, res); });
        //handle_ = node.handle_->create_service<T>(service_name, std::bind(std::forward<Ta>(callback), ptr, std::placeholders::_1, std::placeholders::_2));
#endif
    }

private:
#if MEME_ROS_VERSION == 1
    ros::ServiceServer handle_;
#elif MEME_ROS_VERSION == 2
    typename rclcpp::Service<T>::SharedPtr handle_;
#endif
};

template<typename T>
class Client {
public:
    enum class Status {
        SUCCESSFUL, SUCCESSFUL_WITH_RETRIES, FAILURE
    };

    explicit Client(const std::string &service_name)
            : name_(service_name) {
        auto &node = Node::get();

        printf("[Client '%s'] Create\n", service_name.c_str());

#if MEME_ROS_VERSION == 1
        handle_ = node.handle_.serviceClient<T>(service_name, true);
#elif MEME_ROS_VERSION == 2
        handle_ = node.handle_->create_client<T>(service_name);
#endif
    }

    Status call(const mementar::compat::RequestType<T> &req, mementar::compat::ResponseType<T> &res) {
        printf("[Client '%s'] Call\n", name_.c_str());

        using namespace std::chrono_literals;
        auto status = Status::FAILURE;

#if MEME_ROS_VERSION == 1
        T srv;
        srv.request = req;
        if (!handle_.call(srv))
        {
          auto& node = Node::get();
          handle_ = node.handle_.serviceClient<T>(name_, true);
          if (handle_.call(srv))
          {
            status = Status::SUCCESSFUL_WITH_RETRIES;
            res = srv.response;
          }
        }
        else
        {
          status = Status::SUCCESSFUL;
          res = srv.response;
        }

#elif MEME_ROS_VERSION == 2
        if (!handle_->wait_for_service(5s)) {
            return status;
        }

        auto future = handle_->async_send_request(req);

        if (future.wait_for(5s) == std::future_status::ready) {
            status = Status::SUCCESSFUL;
            res = future.get();
        }
#endif
        return status;
    }

    bool wait(double timeout) {
        printf("[Client '%s'] Wait\n", name_.c_str());
#if MEME_ROS_VERSION == 1
        return handle_.waitForExistence(ros::Duration(timeout));
#elif MEME_ROS_VERSION == 2
        return handle_->wait_for_service(std::chrono::duration<double>(timeout));
#endif
    }

private:
    std::string name_;
#if MEME_ROS_VERSION == 1
    ros::ServiceClient handle_;
#elif MEME_ROS_VERSION == 2
    typename rclcpp::Client<T>::SharedPtr handle_;
#endif
};

} // namespace meme_ros

} // namespace mementar::compat

#endif // COMPAT_MEME_ROS_H