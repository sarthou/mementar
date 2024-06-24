#ifndef MEMENTAR_BINARYMANAGER_H
#define MEMENTAR_BINARYMANAGER_H

#include <fstream>
#include <iostream>
#include <streambuf>
#include <vector>

namespace mementar {

  class BinaryManager
  {
  public:
    explicit BinaryManager(const std::string& extension) : extension_(extension)
    {}

    void displayCompressionRate(size_t in_size, size_t out_size)
    {
      std::cout << "Compression rate : " << (1 - ((float)out_size / (float)in_size)) * 100.0f << std::endl;
    }

    void saveToFile(std::vector<char>& data, const std::string& file_name)
    {
      std::ofstream outfile;
      outfile.open(file_name + "." + extension_, std::ios::binary | std::ios::out);
      outfile.write(reinterpret_cast<char*>(data.data()), (long)data.size());
      outfile.close();
    }

    bool readBinaryFile(std::vector<char>& data, const std::string& file_name)
    {
      std::ifstream infile(file_name, std::ios::binary | std::ios::ate);
      std::streamsize size = infile.tellg();
      infile.seekg(0, std::ios::beg);

      data = std::vector<char>(size);
      if(infile.read(data.data(), size))
        return true;
      else
        return false;
    }

    std::string readBinaryFile(const std::string& file_name)
    {
      std::ifstream infile(file_name, std::ios::binary);
      std::string res((std::istreambuf_iterator<char>(infile)),
                      std::istreambuf_iterator<char>());
      return res;
    }

  private:
    std::string extension_;
  };

} // namespace mementar

#endif // MEMENTAR_BINARYMANAGER_H
