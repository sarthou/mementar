#include <chrono>
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */
#include <iostream>
#include <string>

#include "mementar/core/LtManagement/EpisodicTree/ArchivedLeafNode.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

using namespace std::chrono;

int main()
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  mementar::ArchivedLeafNode archived_node("/home/gsarthou/Desktop/test", 4);

  std::cout << " *************" << std::endl;
  for(mementar::SoftPoint::Ttime i = 0.f; i < 400000.f; i++)
  {
    archived_node.insert(new mementar::Fact(mementar::Triplet("bob", "hasValue", std::to_string(i)), i));
    if(i == 250000)
      archived_node.newSession();
  }
  std::cout << " *************" << std::endl;

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "took " << time_span.count() << std::endl;

  std::cout << "will remove" << std::endl;
  // archived_node.remove(mementar::LinkedFact<time_t>(102, "bob", "hasValue", std::to_string(102)));
  archived_node.remove(archived_node.find(102)->getData()[0]);
  std::cout << "removed" << std::endl;
  archived_node.insert(new mementar::Fact(mementar::Triplet("bob", "hasValue", std::to_string(0)), 102));
  std::cout << "inserted" << std::endl;

  std::cout << " *************" << std::endl;
  /*//archived_node.display(150000);

  std::cout << "first key = " << archived_node.getFirst()->getKey() << std::endl;

  std::cout << "find key = " << archived_node.find(50)->getKey() << std::endl;
  std::cout << "findNear key = " << archived_node.findNear(102)->getKey() << std::endl;
*/
  // mementar::CompressedLeaf<size_t> leaf(&tree, "/home/gsarthou/Desktop");

  return 0;
}
