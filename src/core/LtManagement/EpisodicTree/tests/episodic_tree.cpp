#include <chrono>
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */
#include <iostream>
#include <string>

#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafNode.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

using namespace std::chrono;

int main()
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  {
    mementar::CompressedLeafNode compressed_node("/home/gsarthou/Desktop/test");

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "took " << time_span.count() << " to load" << std::endl;

    std::cout << " *************" << std::endl;
    for(mementar::SoftPoint::Ttime i = 0.f; i < 400000.f; i++)
    {
      // std::cout << i << std::endl;
      compressed_node.insert(new mementar::Fact(mementar::Triplet("bob", "hasValue", std::to_string(i)), i));
      // usleep(1);
    }
    std::cout << " *************" << std::endl;

    high_resolution_clock::time_point t3 = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t3 - t2);
    std::cout << "took " << time_span.count() << " to insert" << std::endl;

    compressed_node.remove(compressed_node.find(102)->getData()[0]);
    compressed_node.insert(new mementar::Fact(mementar::Triplet("bob", "hasValue", std::to_string(0)), 102));

    std::cout << " *************" << std::endl;

    high_resolution_clock::time_point t4 = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t4 - t3);
    std::cout << "took " << time_span.count() << " to modify" << std::endl;
  }

  high_resolution_clock::time_point t5 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t5 - t1);
  std::cout << "took " << time_span.count() << " for all process" << std::endl;

  /*//compressed_node.display(150000);

  std::cout << "first key = " << compressed_node.getFirst()->getKey() << std::endl;

  std::cout << "find key = " << compressed_node.find(50)->getKey() << std::endl;
  std::cout << "findNear key = " << compressed_node.findNear(102)->getKey() << std::endl;
*/
  // mementar::CompressedLeaf<size_t> leaf(&tree, "/home/gsarthou/Desktop");

  return 0;
}
