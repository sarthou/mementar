#include <chrono>
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define NEW_V

#ifdef NEW_V
#include "mementar/core/LtManagement/archiving_compressing/compressing/Huffman.h"
#else
#include "mementar/core/LtManagement/archiving_compressing/compressing/Huffman_old.h"
#endif

using namespace std::chrono;

float testHuffman(size_t nb, const std::string& input_file)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(size_t i = 0; i < nb; i++)
  {
#ifdef NEW_V
    mementar::Huffman huff;
#else
    mementar::Huffman_ huff;
#endif
    std::vector<char> data;
    if(huff.readBinaryFile(data, input_file))
    {
      high_resolution_clock::time_point t11 = high_resolution_clock::now();
#ifdef NEW_V
      size_t tree_size = huff.setTree(data);
      high_resolution_clock::time_point t12 = high_resolution_clock::now();
      data = std::vector<char>(data.begin() + (int)tree_size, data.end());
      std::string out = huff.getFile(data);
#else
      std::string out;
      size_t tree_size = huff.setTree(data);
      high_resolution_clock::time_point t12 = high_resolution_clock::now();
      data = std::vector<char>(data.begin() + tree_size, data.end());
      huff.getFile(data, out);
#endif
      high_resolution_clock::time_point t13 = high_resolution_clock::now();

      duration<double> time_span = duration_cast<duration<double>>(t12 - t11);
      std::cout << "(2 - 1) = " << time_span.count() * 1000 << std::endl;
      time_span = duration_cast<duration<double>>(t13 - t12);
      std::cout << "(3 - 2) = " << time_span.count() * 1000 << std::endl;

      if(i == 0)
      {
        std::ofstream myfile;
        myfile.open("../tests_files/out/" + std::to_string(data.size()) + ".txt");
        myfile << out;
        myfile.close();
      }
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

  float time_252 = testHuffman(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_252_1MB.mhu");
  float time_10 = testHuffman(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_10_9MB.mhu");
  float time_6 = testHuffman(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_6_5MB.mhu");

  float mean = (time_252 + time_10 + time_6) / 3.0f;

  std::cout << "252 = " << time_252 << std::endl;
  std::cout << "10 = " << time_10 << std::endl;
  std::cout << "6 = " << time_6 << std::endl;
  std::cout << "mean = " << mean << std::endl;

  return 0;
}
