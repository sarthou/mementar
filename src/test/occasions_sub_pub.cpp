#include <iostream>
#include <chrono>

#include "ros/ros.h"

#include "ontologenius/OntologyManipulator.h"

#include "include/mementar/API/mementar/TimelineManipulator.h"
#include "include/mementar/API/mementar/OccasionsSubscriber.h"

using namespace std::chrono;

high_resolution_clock::time_point t1, t2;

void callback_1(const mementar::Fact& fct)
{
  std::cout << "[CB1] " << fct() << std::endl;
}

void callback_2(const mementar::Fact& fct)
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
  ros::init(argc, argv, "occasions_sub_pub");

  ros::NodeHandle n;
  onto::OntologyManipulator onto;

  mementar::TimelineManipulator manip(&n);
  manip.waitInit();

  std::cout << "init" << std::endl;

  mementar::OccasionsSubscriber sub1(&callback_1, true);
  sub1.subscribe(mementar::Fact("bob", "eat", "?"), 2);
  mementar::OccasionsSubscriber sub2(&callback_2, true);
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
    mementar::OccasionsSubscriber onto_sub(&ontoCallback, true);
    onto_sub.subscribe(mementar::Fact("onto", "isA", "Ontology"), 1);

    for(size_t i = 0; i < 100; i++)
      r.sleep();

    onto.feeder.addConcept("onto");
    onto.feeder.addProperty("onto", "isA", "Ontology");
    t1 = high_resolution_clock::now();

    while(!onto_sub.end() && ros::ok())
      r.sleep();

    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "callback in " << time_span.count() << std::endl;

    onto_sub.subscribe(mementar::Fact("oro", "isUnder", "onto"), 1);
    onto.feeder.addProperty("onto", "isOn", "oro");
    t1 = high_resolution_clock::now();

    while(!onto_sub.end() && ros::ok())
      r.sleep();

    time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "callback in " << time_span.count() << std::endl;
    onto.feeder.removeProperty("onto", "isOn", "oro");
  }

  return 0;
}
