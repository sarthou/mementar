#ifndef MEMENTAR_ROSINTERFACE_H
#define MEMENTAR_ROSINTERFACE_H

#include <shared_mutex>
#include <string>

#include "mementar/compat/ros.h"
#include "ontologenius/OntologyManipulator.h"
#include "ontologenius/compat/ros.h"

// #include "mementar/core/LtManagement/EpisodicTree/ArchivedLeafNode.h"
#include "mementar/core/Occasions/OccasionsManager.h"
#include "mementar/core/Parametrization/Configuration.h"
#include "mementar/core/feeder/Feeder.h"
#include "mementar/core/feeder/FeederEcho.h"
#include "mementar/core/memGraphs/Timeline.h"

namespace mementar {

  struct Param_t
  {
    std::string base;

    std::string operator()() const { return base; }
  };

  class RosInterface
  {
  public:
    RosInterface(const std::string& directory, const std::string& configuration_file, size_t order = 10, const std::string& name = "");
    ~RosInterface();

    void run();
    void stop() { run_ = false; }
    bool isRunning() const { return run_; }

    void lock();
    void release();

  private:
    std::string directory_;
    Configuration configuration_;
    size_t order_;
    onto::OntologyManipulator onto_;

    Timeline* timeline_;
    Feeder feeder_;
    FeederEcho feeder_echo_;
    OccasionsManager occasions_;

    std::string name_;
    std::atomic<bool> run_;
    std::shared_timed_mutex mut_;
    std::mutex feeder_mutex_;

    void reset();

    void knowledgeCallback(const compat::mem_ros::MessageWrapper<compat::StampedString>& msg);
    void stampedKnowledgeCallback(const compat::mem_ros::MessageWrapper<compat::StampedString>& msg);
    void explanationKnowledgeCallback(const compat::mem_ros::MessageWrapper<compat::MementarExplanation>& msg);
    void actionKnowledgeCallback(const compat::mem_ros::MessageWrapper<compat::MementarAction>& msg);
    void ontoStampedKnowledgeCallback(const compat::mem_ros::MessageWrapper<ontologenius::compat::OntologeniusStampedString>& msg);
    void ontoExplanationKnowledgeCallback(const compat::mem_ros::MessageWrapper<ontologenius::compat::OntologeniusExplanation>& msg);

    bool managerInstanceHandle(compat::mem_ros::ServiceWrapper<compat::MementarService::Request>& req,
                               compat::mem_ros::ServiceWrapper<compat::MementarService::Response>& res);

    bool actionHandle(compat::mem_ros::ServiceWrapper<compat::MementarService::Request>& req,
                      compat::mem_ros::ServiceWrapper<compat::MementarService::Response>& res);

    bool factHandle(compat::mem_ros::ServiceWrapper<compat::MementarService::Request>& req,
                    compat::mem_ros::ServiceWrapper<compat::MementarService::Response>& res);

    void feedThread();

    void removeUselessSpace(std::string& text);
    void set2string(const std::unordered_set<std::string>& word_set, std::string& result);
    void set2vector(const std::unordered_set<std::string>& word_set, std::vector<std::string>& result);
    Param_t getParams(const std::string& param);

    std::string getTopicName(const std::string& topic_name)
    {
      return getTopicName(topic_name, name_);
    }

    std::string getTopicName(const std::string& topic_name, const std::string& onto_name)
    {
      return (onto_name.empty()) ? "/mementar/" + topic_name : "/mementar/" + topic_name + "/" + onto_name;
    }

    std::string getOntoTopicName(const std::string& topic_name)
    {
      return getOntoTopicName(topic_name, name_);
    }

    std::string getOntoTopicName(const std::string& topic_name, const std::string& onto_name)
    {
      return (onto_name.empty()) ? "/ontologenius/" + topic_name : "/ontologenius/" + topic_name + "/" + onto_name;
    }

    double rosTime2Double(double s, int ns);
  };

} // namespace mementar

#endif // MEMENTAR_ROSINTERFACE_H
