#include <chrono>
#include <cstddef>
#include <iostream>
#include <thread>

#include "mementar/API/mementar/Fact.h"
#include "mementar/API/mementar/OccasionsSubscriber.h"
#include "mementar/API/mementar/TimelineManipulator.h"
#include "mementar/compat/ros.h"
#include "ontologenius/OntologyManipulator.h"

using namespace std::chrono;

high_resolution_clock::time_point t1, t2;

void callback1(const mementar::Fact& fct)
{
  std::cout << "[CB1] " << fct() << std::endl;
}

void callback2(const mementar::Fact& fct)
{
  std::cout << "[CB2] " << fct() << std::endl;
}

void ontoCallback(const mementar::Fact& fct)
{
  std::cout << "[onto] " << fct() << std::endl;
  t2 = high_resolution_clock::now();
}

int main(int argc, char** argv)
{
  mementar::compat::mem_ros::Node::init(argc, argv, "occasions_sub_pub");
  std::thread th([]() { mementar::compat::mem_ros::Node::get().spin(); });

  onto::OntologyManipulator onto;

  mementar::TimelineManipulator manip;
  manip.waitInit();

  std::cout << "init" << std::endl;

  mementar::OccasionsSubscriber sub1(&callback1);
  sub1.subscribe(mementar::Fact("bob", "eat", "?"), 2);
  mementar::OccasionsSubscriber sub2(&callback2);
  sub2.subscribe(mementar::Fact("max", "eat", "?"), 3);
  sub2.subscribe(mementar::Fact("bob", "eat", "?"), 4);

  std::cout << "sub" << std::endl;

  size_t cpt = 0;
  ros::Rate r(100);
  /*while((!sub1.end() || !sub2.end()) && ros::ok())
  {
    cpt++;
    if(cpt > 100)
    {
      cpt = 0;
      manip.fact_feeder.insert(mementar::Fact("bob", "eat", "blop"));
      manip.fact_feeder.insert(mementar::Fact("max", "eat", "blop"));
    }
    ros::spinOnce();
    r.sleep();
  }*/

  {
    onto.feeder.waitConnected();
    mementar::OccasionsSubscriber onto_sub(&ontoCallback);
    onto_sub.subscribe(mementar::Fact("onto", "isA", "Ontology"), 1);

    for(size_t i = 0; i < 100; i++)
      r.sleep();

    onto.feeder.addConcept("onto");
    onto.feeder.addProperty("onto", "isA", "Ontology");
    onto.feeder.waitUpdate(1500);
    t1 = high_resolution_clock::now();

    while(!onto_sub.end() && mementar::compat::mem_ros::Node::ok())
      r.sleep();

    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "callback in " << time_span.count() << std::endl;

    onto_sub.subscribe(mementar::Fact("oro", "isUnder", "onto"), 1);
    onto.feeder.addProperty("onto", "isOnTopOf", "oro");
    onto.feeder.waitUpdate(1500);
    t1 = high_resolution_clock::now();

    while(!onto_sub.end() && mementar::compat::mem_ros::Node::ok())
      r.sleep();

    time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "callback in " << time_span.count() << std::endl;
    onto.feeder.removeProperty("onto", "isOnTopOf", "oro");
    onto.feeder.waitUpdate(1500);
  }

  mementar::compat::mem_ros::Node::shutdown();
  th.join();

  return 0;
}
