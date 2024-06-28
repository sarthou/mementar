#include <atomic>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "mementar/API/mementar/ActionsSubscriber.h"
#include "mementar/API/mementar/OccasionsSubscriber.h"
#include "mementar/API/mementar/TimelineManipulator.h"
#include "ontologenius/OntologyManipulator.h"

mementar::TimelineManipulator* time_ptr;

std::atomic<size_t> cpt_fact_start;
std::atomic<size_t> cpt_fact_end;
std::atomic<size_t> cpt_act_start;
std::atomic<size_t> cpt_act_end;

void factCallbackStart(const mementar::Fact& fct)
{
  cpt_fact_start++;
}

void factCallbackEnd(const mementar::Fact& fct)
{
  cpt_fact_end++;
}

void actionCallbackStart(const std::string& act)
{
  cpt_act_start++;
}

void actionCallbackEnd(const std::string& act)
{
  cpt_act_end++;
}

TEST(action_pub_sub_tests, TimelineManipulator_fact_start_subscriber)
{
  ros::Rate r(0.9);
  mementar::OccasionsSubscriber sub(&factCallbackStart);
  sub.subscribe(mementar::Fact("act_0", "_", "start"), 1);
  cpt_fact_start = 0;
  cpt_fact_end = 0;

  r.sleep();
  for(size_t i = 0; i < 2; i++)
  {
    time_ptr->action_feeder.insert("act_" + std::to_string(i));
    time_ptr->action_feeder.insert("other_act_" + std::to_string(i));
  }
  for(size_t i = 0; i < 2; i++)
  {
    time_ptr->action_feeder.insertEnd("act_" + std::to_string(i));
    time_ptr->action_feeder.insertEnd("other_act_" + std::to_string(i));
  }
  r.sleep();

  EXPECT_EQ(cpt_fact_start, 1);
  EXPECT_EQ(cpt_fact_end, 0);
}

TEST(action_pub_sub_tests, TimelineManipulator_fact_end_subscriber)
{
  ros::Rate r(0.9);
  mementar::OccasionsSubscriber sub(&factCallbackEnd);
  sub.subscribe(mementar::Fact("act_10", "_", "end"), 1);
  cpt_fact_start = 0;
  cpt_fact_end = 0;

  r.sleep();
  for(size_t i = 10; i < 12; i++)
  {
    time_ptr->action_feeder.insert("act_" + std::to_string(i));
    time_ptr->action_feeder.insert("other_act_" + std::to_string(i));
  }
  for(size_t i = 10; i < 12; i++)
  {
    time_ptr->action_feeder.insertEnd("act_" + std::to_string(i));
    time_ptr->action_feeder.insertEnd("other_act_" + std::to_string(i));
  }
  r.sleep();

  EXPECT_EQ(cpt_fact_start, 0);
  EXPECT_EQ(cpt_fact_end, 1);
}

TEST(action_pub_sub_tests, TimelineManipulator_action_start_subscriber)
{
  ros::Rate r(0.9);
  mementar::ActionsSubscriber sub(&actionCallbackStart);
  sub.subscribeToStart("act_20", 1);
  cpt_act_start = 0;
  cpt_act_end = 0;

  r.sleep();
  for(size_t i = 20; i < 22; i++)
  {
    time_ptr->action_feeder.insert("act_" + std::to_string(i));
    time_ptr->action_feeder.insert("other_act_" + std::to_string(i));
  }
  for(size_t i = 20; i < 22; i++)
  {
    time_ptr->action_feeder.insertEnd("act_" + std::to_string(i));
    time_ptr->action_feeder.insertEnd("other_act_" + std::to_string(i));
  }
  r.sleep();

  EXPECT_EQ(cpt_act_start, 1);
  EXPECT_EQ(cpt_act_end, 0);
}

TEST(action_pub_sub_tests, TimelineManipulator_action_end_subscriber)
{
  ros::Rate r(0.9);
  mementar::ActionsSubscriber sub(&actionCallbackEnd);
  sub.subscribeToEnd("act_30", 1);
  cpt_act_start = 0;
  cpt_act_end = 0;

  r.sleep();
  for(size_t i = 30; i < 32; i++)
  {
    time_ptr->action_feeder.insert("act_" + std::to_string(i));
    time_ptr->action_feeder.insert("other_act_" + std::to_string(i));
  }
  for(size_t i = 30; i < 32; i++)
  {
    time_ptr->action_feeder.insertEnd("act_" + std::to_string(i));
    time_ptr->action_feeder.insertEnd("other_act_" + std::to_string(i));
  }
  r.sleep();

  EXPECT_EQ(cpt_act_start, 0);
  EXPECT_EQ(cpt_act_end, 1);
}

TEST(action_pub_sub_tests, TimelineManipulator_action_start_end_subscriber)
{
  ros::Rate r(0.9);
  mementar::ActionsSubscriber sub_s(&actionCallbackStart);
  sub_s.subscribeToStart("act_40", 1);
  mementar::ActionsSubscriber sub_e(&actionCallbackEnd);
  sub_e.subscribeToEnd("act_40", 1);
  cpt_act_start = 0;
  cpt_act_end = 0;

  r.sleep();
  for(size_t i = 40; i < 42; i++)
  {
    time_ptr->action_feeder.insert("act_" + std::to_string(i), time(0), time(0) + 2);
    time_ptr->action_feeder.insert("other_act_" + std::to_string(i), time(0), time(0) + 2);
  }
  r.sleep();

  EXPECT_EQ(cpt_act_start, 1);
  EXPECT_EQ(cpt_act_end, 1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_pub_sub_tests");

  // std::thread th([]() { mementar::compat::mem_ros::Node::get().spin(); });
  std::thread th([]() { ros::spin(); });

  onto::OntologyManipulator onto;
  mementar::TimelineManipulator timeline;
  timeline.waitInit();
  time_ptr = &timeline;

  onto.close();

  testing::InitGoogleTest(&argc, argv);

  // mementar::compat::mem_ros::Node::shutdown();
  // th.join();

  return RUN_ALL_TESTS();
}
