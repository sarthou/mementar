#include <atomic>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "include/mementar/API/mementar/OccasionsSubscriber.h"
#include "include/mementar/API/mementar/TimelineManipulator.h"
#include "ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;
mementar::TimelineManipulator* time_ptr;
std::atomic<bool> done;

std::atomic<size_t> cpt_abc;
std::atomic<size_t> cpt_def;
std::atomic<size_t> cpt_cIsUndera;

void factCallbackABC(const mementar::Fact& fct)
{
  cpt_abc++;
}

void factCallbackDEF(const mementar::Fact& fct)
{
  cpt_def++;
}

void factCallbackCisUnderA(const mementar::Fact& fct)
{
  cpt_cIsUndera++;
}

TEST(fact_pub_sub_tests, TimelineManipulator_one_subscriber)
{
  ros::Rate r(0.9);
  mementar::OccasionsSubscriber sub1(&factCallbackABC);
  sub1.subscribe(mementar::Fact("a", "b", "c"), 3);
  cpt_abc = 0;
  cpt_def = 0;

  r.sleep();
  for(size_t i = 0; i < 5; i++)
  {
    time_ptr->fact_feeder.insert(mementar::Fact("a", "b", "c"));
    time_ptr->fact_feeder.insert(mementar::Fact("d", "e", "f"));
  }
  r.sleep();

  EXPECT_EQ(cpt_abc, 3);
  EXPECT_EQ(cpt_def, 0);
}

TEST(fact_pub_sub_tests, TimelineManipulator_two_subscriber)
{
  ros::Rate r(0.9);
  mementar::OccasionsSubscriber sub1(&factCallbackABC);
  sub1.subscribe(mementar::Fact("a", "b", "c"), 4);
  mementar::OccasionsSubscriber sub2(&factCallbackDEF);
  sub2.subscribe(mementar::Fact("d", "e", "f"), 1);
  cpt_abc = 0;
  cpt_def = 0;

  r.sleep();
  for(size_t i = 0; i < 5; i++)
  {
    time_ptr->fact_feeder.insert(mementar::Fact("a", "b", "c"));
    time_ptr->fact_feeder.insert(mementar::Fact("d", "e", "f"));
  }
  r.sleep();

  EXPECT_EQ(cpt_abc, 4);
  EXPECT_EQ(cpt_def, 1);
}

TEST(fact_pub_sub_tests, TimelineManipulator_three_subscriber)
{
  ros::Rate r(0.9);
  mementar::OccasionsSubscriber sub1(&factCallbackABC);
  sub1.subscribe(mementar::Fact("a", "b", "c"), 4);
  sub1.subscribe(mementar::Fact("d", "e", "f"), 2);
  mementar::OccasionsSubscriber sub2(&factCallbackDEF);
  sub2.subscribe(mementar::Fact("d", "e", "f"), 1);
  cpt_abc = 0;
  cpt_def = 0;

  r.sleep();
  for(size_t i = 0; i < 5; i++)
  {
    time_ptr->fact_feeder.insert(mementar::Fact("a", "b", "c"));
    time_ptr->fact_feeder.insert(mementar::Fact("d", "e", "f"));
  }
  r.sleep();

  EXPECT_EQ(cpt_abc, 6);
  EXPECT_EQ(cpt_def, 1);
}

TEST(fact_pub_sub_tests, TimelineManipulator_pattern_subscriber)
{
  ros::Rate r(0.9);
  mementar::OccasionsSubscriber sub1(&factCallbackABC);
  sub1.subscribe(mementar::Fact("a", "b", "?"), 10);
  cpt_abc = 0;
  cpt_def = 0;

  r.sleep();
  for(size_t i = 0; i < 5; i++)
  {
    time_ptr->fact_feeder.insert(mementar::Fact("a", "b", "c"));
    time_ptr->fact_feeder.insert(mementar::Fact("a", "b", "d"));
  }
  r.sleep();

  EXPECT_EQ(cpt_abc, 10);
  EXPECT_EQ(cpt_def, 0);
}

TEST(fact_pub_sub_tests, OntologyManipulator_one_subscriber)
{
  ros::Rate r(0.9);
  onto_ptr->feeder.addInheritage("a", "cube");
  onto_ptr->feeder.removeProperty("a", "b", "c");
  onto_ptr->feeder.waitUpdate(1500);

  mementar::OccasionsSubscriber sub1(&factCallbackABC);
  sub1.subscribe(mementar::Fact("a", "b", "c"), 1);
  cpt_abc = 0;
  cpt_def = 0;

  r.sleep();
  onto_ptr->feeder.addProperty("a", "b", "c");
  r.sleep();

  EXPECT_EQ(cpt_abc, 1);
  EXPECT_EQ(cpt_def, 0);
}

TEST(fact_pub_sub_tests, OntologyManipulator_inv_subscriber)
{
  ros::Rate r(0.9);
  onto_ptr->feeder.addInheritage("a", "cube");
  onto_ptr->feeder.removeProperty("a", "isOn", "c");
  onto_ptr->feeder.waitUpdate(1500);

  mementar::OccasionsSubscriber sub1(&factCallbackCisUnderA);
  sub1.subscribe(mementar::Fact("c", "isUnder", "a"), 1);
  cpt_cIsUndera = 0;

  r.sleep();
  onto_ptr->feeder.addProperty("a", "isOn", "c");
  r.sleep();

  EXPECT_EQ(cpt_cIsUndera, 1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fact_pub_sub_tests");

  // std::thread th([]() { mementar::compat::mem_ros::Node::get().spin(); });
  std::thread th([]() { ros::spin(); });

  onto::OntologyManipulator onto;
  onto_ptr = &onto;
  mementar::TimelineManipulator timeline;
  timeline.waitInit();
  time_ptr = &timeline;

  onto.close();

  testing::InitGoogleTest(&argc, argv);

  // mementar::compat::mem_ros::Node::shutdown();
  // th.join();

  return RUN_ALL_TESTS();
}
