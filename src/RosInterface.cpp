#include "mementar/RosInterface.h"

#include <algorithm>
#include <cstddef>
#include <ctime>
#include <filesystem>
#include <iterator>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "mementar/compat/ros.h"
#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"
#include "mementar/core/memGraphs/Branchs/ValuedNode.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Timeline.h"
#include "mementar/core/utility/error_code.h"
#include "mementar/graphical/Display.h"
#include "mementar/graphical/timeline/CsvSaver.h"
#include "mementar/graphical/timeline/TimelineDrawer.h"
#include "mementar/utils/String.h"

namespace mementar {

  RosInterface::RosInterface(const std::string& directory,
                             const std::string& configuration_file,
                             size_t order,
                             const std::string& name) : order_(order),
                                                        onto_(name),
                                                        feeder_(&onto_),
                                                        feeder_echo_(getTopicName("echo", name)),
                                                        occasions_(&onto_, name),
                                                        run_(true)
  {
    onto_.close();

    if(directory != "none")
    {
      directory_ = directory;
      if(name.empty())
        directory_ += "/mementar";
      else
        directory_ += "/" + name;

      std::filesystem::create_directories(directory_);
    }

    if(configuration_file != "none")
    {
      if(configuration_.read(configuration_file))
      {
        if(configuration_.exist("whitelist"))
        {
          if(feeder_.setWhitelist(configuration_["whitelist"].value()) == false)
            Display::error("A whitelist can not be setted while a blacklist is used");
          else
            Display::info("A whitelist has been setted");
        }
        if(configuration_.exist("blacklist"))
        {
          if(feeder_.setBlacklist(configuration_["blacklist"].value()) == false)
            Display::error("A blacklist can not be setted while a whitelist is used");
          else
            Display::info("A blacklist has been setted");
        }
      }
      else
        Display::error("Fail to load configuartion file : " + configuration_file);
    }

    timeline_ = new Timeline();
    feeder_.link(timeline_);

    name_ = name;
  }

  RosInterface::~RosInterface()
  {
    delete timeline_;
  }

  void RosInterface::run()
  {
    std::vector<compat::mem_ros::Subscriber<compat::StampedString>> str_subscribers;
    str_subscribers.emplace_back(getTopicName("insert_fact"), 1000, &RosInterface::knowledgeCallback, this);
    str_subscribers.emplace_back(getTopicName("insert_fact_stamped"), 1000, &RosInterface::stampedKnowledgeCallback,
                                 this);

    compat::mem_ros::Subscriber<compat::MementarExplanation> explanation_knowledge_subscriber(
      getTopicName("insert_fact_explanations"), 1000, &RosInterface::explanationKnowledgeCallback, this);
    compat::mem_ros::Subscriber<compat::MementarAction> action_knowledge_subscriber(getTopicName("insert_action"),
                                                                                    1000,
                                                                                    &RosInterface::actionKnowledgeCallback,
                                                                                    this);

    compat::mem_ros::Subscriber<ontologenius::compat::OntologeniusStampedString>
      onto_stamped_knowledge_subscriber(getOntoTopicName("insert_echo"), 1000,
                                        &RosInterface::ontoStampedKnowledgeCallback, this);
    compat::mem_ros::Subscriber<ontologenius::compat::OntologeniusExplanation>
      onto_explanation_knowledge_subscriber(getOntoTopicName("insert_explanations"), 1000,
                                            &RosInterface::ontoExplanationKnowledgeCallback, this);

    // Start up ROS service with callbacks

    std::vector<compat::mem_ros::Service<compat::MementarService>> services;
    services.emplace_back(getTopicName("manage_instance"), &RosInterface::managerInstanceHandle, this);
    services.emplace_back(getTopicName("action"), &RosInterface::actionHandle, this);
    services.emplace_back(getTopicName("fact"), &RosInterface::factHandle, this);

    feeder_.setCallback([this](ContextualizedFact* fact) {
      this->occasions_.add(*fact);
      this->feeder_echo_.add(fact);
    });
    std::thread occasions_thread(&OccasionsManager::run, &occasions_);
    std::thread feed_thread(&RosInterface::feedThread, this);

    // ROS_DEBUG("%s mementar ready", name_.c_str());

    while(compat::mem_ros::Node::ok() && isRunning())
    {
      // todo
      // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }

    occasions_.stop();
    occasions_thread.join();
    feed_thread.join();
  }

  void RosInterface::reset()
  {
    mut_.lock();
    delete timeline_;

    if(directory_.empty() == false)
    {
      std::filesystem::remove_all(directory_);
      std::filesystem::create_directories(directory_);
    }

    ValuedNode::table_.reset();
    timeline_ = new Timeline();
    feeder_.link(timeline_);
    mut_.unlock();

    Display::info("Timeline of " + name_ + " has been reset.");
  }

  void RosInterface::lock()
  {
    feeder_mutex_.lock();
  }

  void RosInterface::release()
  {
    feeder_mutex_.unlock();
  }

  /***************
   *
   * Callbacks
   *
   ****************/

  double RosInterface::rosTime2Float(double s, int ns)
  {
    ns = ns / 100000000;
    double res = (double)ns / 10.;
    if(res < 0.25)
      return s;
    else if(res < 0.5)
      return s + 0.25;
    else if(res < 0.75)
      return s + 0.5;
    else
      return s + 0.75;
  }

  void RosInterface::knowledgeCallback(compat::mem_ros::MessageWrapper<compat::StampedString> msg)
  {
    feeder_.storeFact(msg->data, std::time(nullptr));
  }

  void RosInterface::stampedKnowledgeCallback(compat::mem_ros::MessageWrapper<compat::StampedString> msg)
  {
    feeder_.storeFact(msg->data, rosTime2Float(msg->stamp.seconds, msg->stamp.nanoseconds));
  }

  void RosInterface::explanationKnowledgeCallback(compat::mem_ros::MessageWrapper<compat::MementarExplanation> msg)
  {
    feeder_.storeFact(msg->fact, msg->cause);
  }

  void RosInterface::actionKnowledgeCallback(compat::mem_ros::MessageWrapper<compat::MementarAction> msg)
  {
    feeder_.storeAction(msg->name,
                        (msg->start_stamp.seconds != 0) ? rosTime2Float(msg->start_stamp.seconds,
                                                                        msg->start_stamp.nanoseconds) :
                                                          SoftPoint::default_time,
                        (msg->end_stamp.seconds != 0) ? rosTime2Float(msg->end_stamp.seconds,
                                                                      msg->end_stamp.nanoseconds) :
                                                        SoftPoint::default_time);
  }

  void RosInterface::ontoStampedKnowledgeCallback(
    compat::mem_ros::MessageWrapper<ontologenius::compat::OntologeniusStampedString> msg)
  {
    feeder_.storeFact(msg->data, rosTime2Float(msg->stamp.seconds, msg->stamp.nanoseconds));
  }

  void RosInterface::ontoExplanationKnowledgeCallback(
    compat::mem_ros::MessageWrapper<ontologenius::compat::OntologeniusExplanation> msg)
  {
    feeder_.storeFact(msg->fact, msg->cause);
  }

  /***************
   *
   * Services
   *
   ****************/

  bool RosInterface::managerInstanceHandle(compat::mem_ros::ServiceWrapper<compat::MementarService::Request>& req,
                                           compat::mem_ros::ServiceWrapper<compat::MementarService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      res->code = ServiceCode::service_no_error;

      removeUselessSpace(req->action);
      removeUselessSpace(req->param);
      // Param_t params = getParams(req->param);

      /*if(req->action == "newSession")
      {
        mut_.lock_shared();
        tree_->newSession();
        mut_.unlock_shared();
      }
      else */
      if(req->action == "reset")
        reset();
      else if(req->action == "draw")
      {
        TimelineDrawer drawer;
        if(drawer.draw(req->param, timeline_) == false)
          res->code = ServiceCode::service_no_effect;
      }
      else if(req->action == "save")
      {
        CsvSaver saver;
        if(saver.save(req->param, timeline_) == false)
          res->code = ServiceCode::service_no_effect;
      }
      else
        res->code = ServiceCode::service_unknown_action;

      return true;
    }(compat::mem_ros::getServicePointer(req), compat::mem_ros::getServicePointer(res));
  }

  bool RosInterface::actionHandle(compat::mem_ros::ServiceWrapper<compat::MementarService::Request>& req,
                                  compat::mem_ros::ServiceWrapper<compat::MementarService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      res->code = 0;

      removeUselessSpace(req->action);
      removeUselessSpace(req->param);
      Param_t params = getParams(req->param);

      std::unordered_set<std::string> set_res;

      if(req->action == "exist")
      {
        if(timeline_->actions.exist(params()))
        {
          res->values.push_back(params());
        }
      }
      else if(req->action == "getPending")
      {
        set_res = timeline_->actions.getPending();
      }
      else if(req->action == "isPending")
      {
        if(timeline_->actions.isPending(params()))
        {
          res->values.push_back(params());
        }
      }
      else if(req->action == "getStartStamp")
      {
        const auto time = compat::mem_ros::Time(timeline_->actions.getStartStamp(params()));
        res->time_value.seconds = time.seconds();
        res->time_value.nanoseconds = time.nanoseconds();
      }
      else if(req->action == "getEndStamp")
      {
        const auto time = compat::mem_ros::Time(timeline_->actions.getEndStamp(params()));
        res->time_value.seconds = time.seconds();
        res->time_value.nanoseconds = time.nanoseconds();
      }
      else if(req->action == "getDuration")
      {
        const auto time = compat::mem_ros::Time(timeline_->actions.getDuration(params()));
        res->time_value.seconds = time.seconds();
        res->time_value.nanoseconds = time.nanoseconds();
      }
      else if(req->action == "getStartFact")
      {
        auto fact_name = timeline_->actions.getStartFact(params());

        if(!fact_name.empty())
        {
          res->values.push_back(fact_name);
        }
      }
      else if(req->action == "getEndFact")
      {
        auto fact_name = timeline_->actions.getEndFact(params());

        if(!fact_name.empty())
        {
          res->values.push_back(fact_name);
        }
      }
      else if(req->action == "getFactsDuring")
        set_res = timeline_->actions.getFactsDuring(params());
      else if(req->action == "removeAction")
      {
        if(timeline_->actions.removeAction(params()) == false)
          res->code = ServiceCode::service_no_effect;
      }
      else
        res->code = ServiceCode::service_unknown_action;

      if(res->values.size() == 0)
        set2vector(set_res, res->values);

      return true;
    }(compat::mem_ros::getServicePointer(req), compat::mem_ros::getServicePointer(res));
  }

  bool RosInterface::factHandle(compat::mem_ros::ServiceWrapper<compat::MementarService::Request>& req,
                                compat::mem_ros::ServiceWrapper<compat::MementarService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      res->code = 0;

      removeUselessSpace(req->action);
      removeUselessSpace(req->param);
      Param_t params = getParams(req->param);

      std::unordered_set<std::string> set_res;

      if(req->action == "exist")
      {
        if(timeline_->facts.exist(params()))
          res->values.push_back(params());
      }
      else if(req->action == "isActionPart")
      {
        if(timeline_->facts.isActionPart(params()))
          res->values.push_back(params());
      }
      else if(req->action == "getActionPart")
      {
        auto action_name = timeline_->facts.getActionPart(params());
        if(action_name.empty() == false)
          res->values.push_back(action_name);
      }
      else if(req->action == "getData")
      {
        auto fact_data = timeline_->facts.getData(params());
        if(fact_data.empty() == false)
          res->values.push_back(fact_data);
      }
      else if(req->action == "getStamp")
      {
        const auto time = compat::mem_ros::Time(timeline_->facts.getStamp(params()));
        res->time_value.seconds = time.seconds();
        res->time_value.nanoseconds = time.nanoseconds();
      }
      else
      {
        res->code = ServiceCode::service_unknown_action;
      }

      if(res->values.size() == 0)
        set2vector(set_res, res->values);

      return true;
    }(compat::mem_ros::getServicePointer(req), compat::mem_ros::getServicePointer(res));
  }

  /***************
   *
   * Threads
   *
   ***************/

  void RosInterface::feedThread()
  {
    compat::mem_ros::Publisher<std_msgs_compat::String> feeder_publisher(getTopicName("feeder_notifications"),
                                                                         1000);
    compat::mem_ros::Rate wait(100);

    while((compat::mem_ros::Node::ok()) && (timeline_->isInit() == false) && (run_ == true))
    {
      wait.sleep();
    }

    std_msgs_compat::String msg;
    while(compat::mem_ros::Node::ok() && (run_ == true))
    {
      feeder_mutex_.lock();
      bool run = feeder_.run();
      if(run == true)
      {
        std::vector<std::string> notifications = feeder_.getNotifications();
        for(auto notif : notifications)
        {
          Display::error(notif);
          if(name_.empty() == false)
            notif = "[" + name_ + "]" + notif;
          msg.data = notif;
          feeder_publisher.publish(msg);
        }
        feeder_echo_.publish();
      }
      feeder_mutex_.unlock();

      if(compat::mem_ros::Node::ok() && (run_ == true))
        wait.sleep();
    }
  }

  /***************
   *
   * Utility
   *
   ****************/

  void RosInterface::removeUselessSpace(std::string& text)
  {
    while((text[0] == ' ') && (text.empty() == false))
      text.erase(0, 1);

    while((text[text.size() - 1] == ' ') && (text.empty() == false))
      text.erase(text.size() - 1, 1);
  }

  void RosInterface::set2string(const std::unordered_set<std::string>& word_set, std::string& result)
  {
    for(const std::string& it : word_set)
      result += it + " ";
  }

  void RosInterface::set2vector(const std::unordered_set<std::string>& word_set, std::vector<std::string>& result)
  {
    std::copy(word_set.cbegin(), word_set.cend(), std::back_inserter(result));
  }

  Param_t RosInterface::getParams(const std::string& param)
  {
    Param_t parameters;
    std::vector<std::string> str_params = split(param, " ");

    if(str_params.empty() == false)
      parameters.base = str_params[0];

    bool option_found = false;
    (void)option_found;
    for(size_t i = 1; i < str_params.size(); i++)
    {
      /*if((str_params[i] == "-i") || (str_params[i] == "--take_id"))
      {
        i++;
        bool tmp = false;
        if(str_params[i] == "true")
          tmp = true;

        parameters.take_id = tmp;
        option_found = true;
      }
      else if(option_found)
        Display::warning("[WARNING] unknow parameter \"" + str_params[i] + "\"");
      else*/
      parameters.base += " " + str_params[i];
    }

    return parameters;
  }

} // namespace mementar
