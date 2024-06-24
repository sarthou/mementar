#include <chrono>
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/compressing/LzUncompress.h"

using namespace std::chrono;

float testLz(size_t nb, const std::string& input_file)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(size_t i = 0; i < nb; i++)
  {
    mementar::LzUncompress lz;
    std::vector<char> data;
    std::string out;
    if(lz.readBinaryFile(data, input_file))
      out = lz.uncompress(data);

    if(i == 0)
    {
      std::ofstream myfile;
      myfile.open("../tests_files/out/" + std::to_string(out.size()) + ".txt");
      myfile << out;
      myfile.close();
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

  const size_t nb = 10;

  float time_252 = testLz(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_252_1MB.mlz");
  float time_10 = testLz(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_10_9MB.mlz");
  float time_6 = testLz(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_6_5MB.mlz");

  float mean = (time_252 + time_10 + time_6) / 3.0f;

  std::cout << "252 = " << time_252 << std::endl;
  std::cout << "10 = " << time_10 << std::endl;
  std::cout << "6 = " << time_6 << std::endl;
  std::cout << "mean = " << mean << std::endl;

  return 0;
}
