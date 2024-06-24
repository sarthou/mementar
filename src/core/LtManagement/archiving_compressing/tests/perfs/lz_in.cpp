#include <chrono>
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/compressing/LzCompress.h"

using namespace std::chrono;

float testLz(size_t nb, const std::string& input_file)
{
  std::ifstream t(input_file);
  std::string in((std::istreambuf_iterator<char>(t)),
                 std::istreambuf_iterator<char>());

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(size_t i = 0; i < nb; i++)
  {
    mementar::LzCompress lz_comp;
    std::vector<char> out_vect = lz_comp.compress(in);

    if(i == 0)
    {
      lz_comp.displayCompressionRate(in.size(), out_vect.size());
      lz_comp.saveToFile(out_vect, "../tests_files/out/" + std::to_string(in.size()));
    }
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  return (float)time_span.count() * 1000.f / (float)nb; // return time span in ms
}

int main(int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  const size_t nb = 1;

  float time_252 = testLz(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_252_1MB.txt");
  float time_10 = testLz(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_10_9MB.txt");
  float time_6 = testLz(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_6_5MB.txt");

  float mean = (time_252 + time_10 + time_6) / 3.0f;

  std::cout << "252 = " << time_252 << std::endl;
  std::cout << "10 = " << time_10 << std::endl;
  std::cout << "6 = " << time_6 << std::endl;
  std::cout << "mean = " << mean << std::endl;

  return 0;
}
