#ifndef MEMENTAR_ROSINTERFACE_H
#define MEMENTAR_ROSINTERFACE_H

#include <string>
#include <shared_mutex>

#include "mementar/compat/ros.h"
#include "ontologenius/compat/ros.h"
#include "ontologenius/OntologyManipulator.h"

//#include "mementar/core/LtManagement/EpisodicTree/ArchivedLeafNode.h"
#include "mementar/core/feeder/Feeder.h"
#include "mementar/core/feeder/FeederEcho.h"
#include "mementar/core/memGraphs/Timeline.h"
#include "mementar/core/Occasions/OccasionsManager.h"
#include "mementar/core/Parametrization/Configuration.h"

namespace mementar
{

struct param_t
{
  std::string base;

  std::string operator()() { return base; }
};

class RosInterface
{
public:
  RosInterface(const std::string& directory, const std::string& configuration_file, size_t order = 10, std::string name = "");
  ~RosInterface();

  void run();
  void stop() {run_ = false; }
  inline bool isRunning() { return run_; }

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

  void knowledgeCallback(compat::onto_ros::MessageWrapper<compat::StampedString> msg);
  void stampedKnowledgeCallback(compat::onto_ros::MessageWrapper<compat::StampedString> msg);
  void explanationKnowledgeCallback(compat::onto_ros::MessageWrapper<compat::MementarExplanation> msg);
  void actionKnowledgeCallback(compat::onto_ros::MessageWrapper<compat::MementarAction> msg);
  void ontoStampedKnowledgeCallback(compat::onto_ros::MessageWrapper<ontologenius::compat::OntologeniusStampedString> msg);
  void ontoExplanationKnowledgeCallback(compat::onto_ros::MessageWrapper<ontologenius::compat::OntologeniusExplanation> msg);

  // todo: do not return bool
  bool managerInstanceHandle(compat::onto_ros::ServiceWrapper<compat::MementarService::Request>& req,
                             compat::onto_ros::ServiceWrapper<compat::MementarService::Response>& res);

  bool actionHandle(compat::onto_ros::ServiceWrapper<compat::MementarService::Request>& req,
                    compat::onto_ros::ServiceWrapper<compat::MementarService::Response>& res);

  bool factHandle(compat::onto_ros::ServiceWrapper<compat::MementarService::Request>& req,
                  compat::onto_ros::ServiceWrapper<compat::MementarService::Response>& res);

   void feedThread();

  void removeUselessSpace(std::string& text);
  void set2string(const std::unordered_set<std::string>& word_set, std::string& result);
  void set2vector(const std::unordered_set<std::string>& word_set, std::vector<std::string>& result);
  param_t getParams(const std::string& param);

  std::string getTopicName(const std::string& topic_name)
  {
    return getTopicName(topic_name, name_);
  }

  std::string getTopicName(const std::string& topic_name, const std::string& onto_name)
  {
    return (onto_name == "") ? "/mementar/" + topic_name : "/mementar/" + topic_name + "/" + onto_name;
  }

  std::string getOntoTopicName(const std::string& topic_name)
  {
    return getOntoTopicName(topic_name, name_);
  }

  std::string getOntoTopicName(const std::string& topic_name, const std::string& onto_name)
  {
    return (onto_name == "") ? "/ontologenius/" + topic_name : "/ontologenius/" + topic_name + "/" + onto_name;
  }

  double rosTime2Float(double s, int ns);
};

} // namespace mementar

#endif // MEMENTAR_ROSINTERFACE_H
