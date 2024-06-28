#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"

#include <chrono>
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"

using namespace std::chrono;

enum Action_e
{
  act_list,
  act_extract,
  act_archive
};

int main(int argc, char* argv[])
{
  std::vector<std::string> input_files;
  std::string description_file;
  std::string output_file;
  Action_e action = act_archive;

  for(int i = 1; i < argc; i++)
  {
    if(std::string(argv[i]) == "-i")
    {
      if(i + 1 < argc)
        input_files.emplace_back(argv[++i]);
      else
      {
        std::cout << "expected argument after the -i option" << std::endl;
        return -1;
      }
    }
    else if(std::string(argv[i]) == "-o")
    {
      if(i + 1 < argc)
        output_file = std::string(argv[++i]);
      else
      {
        std::cout << "expected argument after the -o option" << std::endl;
        return -1;
      }
    }
    else if(std::string(argv[i]) == "-d")
    {
      if(i + 1 < argc)
        description_file = std::string(argv[++i]);
      else
      {
        std::cout << "expected argument after the -d option" << std::endl;
        return -1;
      }
    }
    else if(std::string(argv[i]) == "-l")
      action = act_list;
    else if(std::string(argv[i]) == "-x")
      action = act_extract;
    else if(std::string(argv[i]) == "-help")
    {
      std::cout << "Available options are :" << std::endl;
      std::cout << "\t-i : input file" << std::endl;
      std::cout << "\t-o : output file (or output path for extraction mode)" << std::endl;
      std::cout << "\t-d : description file for archiving mode" << std::endl;
      std::cout << "\t-l : list the contents of the input archive" << std::endl;
      std::cout << "\t-x : extract the contents of the input archive" << std::endl;
      return 0;
    }
  }

  if(input_files.empty())
  {
    std::cout << "specify an input file with -i" << std::endl;
    return -1;
  }
  if((output_file.empty()) && (action != act_list))
  {
    std::cout << "specify the output file with -o" << std::endl;
    return -1;
  }
  if((description_file.empty()) && (action == act_archive))
  {
    std::cout << "specify a description_file file with -d" << std::endl;
    return -1;
  }

  ///////////////////////////////////////////////////////////////////
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  if(action == act_archive)
  {
    std::ifstream t(description_file);
    std::string in((std::istreambuf_iterator<char>(t)),
                   std::istreambuf_iterator<char>());
    mementar::Archive arch(in, input_files);

    std::vector<char> data = arch.load();
    arch.saveToFile(data, output_file);
  }
  else if(action == act_extract)
  {
    mementar::Archive arch;
    arch.readBinaryFile(input_files[0]);
    mementar::Header header = arch.getHeader();

    std::ofstream myfile;
    std::string out;

    out = arch.extractDescription(header);
    myfile.open(output_file + "/description.txt");
    myfile << out;
    myfile.close();

    for(size_t i = 0; i < header.input_files_.size(); i++)
    {
      out = arch.extractFile(i, header);
      myfile.open(output_file + "/" + header.input_files_[i].name_);
      myfile << out;
      myfile.close();
    }
  }
  else if(action == act_list)
  {
    mementar::Archive arch;
    arch.readBinaryFile(input_files[0]);
    mementar::Header header = arch.getHeader();
    std::cout << header.toString() << std::endl;
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  ///////////////////////////////////////////////////////////////////

  std::cout << "time = " << time_span.count() << "s" << std::endl;

  return 0;
}
