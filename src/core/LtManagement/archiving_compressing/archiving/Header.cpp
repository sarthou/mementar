#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"

#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGenerator.h"
#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGetter.h"

namespace mementar {

  std::string Header::toString()
  {
    std::string res;
    std::string tmp;
    tmp.resize(100);
    sprintf(const_cast<char*>(tmp.c_str()), "%-50s | %-10s | %-10s\n", "file name", "size", "offset");
    res += tmp;
    tmp = std::string(50 + 3 + 10 + 3 + 10, '=');
    res += tmp + "\n";
    tmp.resize(100);
    sprintf(const_cast<char*>(tmp.c_str()), "%-50s | %10d | %10d\n", description_file_.name_.c_str(), (int)description_file_.size_, (int)description_file_.offset_);
    res += tmp;
    tmp.resize(100);
    sprintf(const_cast<char*>(tmp.c_str()), "%-50s | %-10s | %-10s\n", "", "", "");
    res += tmp;
    for(const auto& file : input_files_)
    {
      tmp.resize(100);
      sprintf(const_cast<char*>(tmp.c_str()), "%-50s | %10d | %10d\n", file.name_.c_str(), (int)file.size_, (int)file.offset_);
      res += tmp;
    }

    return res;
  }

  size_t Header::encodedSize()
  {
    size_t nb_bit = 0;
    nb_bit += 8 + description_file_.name_.size() * 7 + 31 + 31;
    nb_bit += 16;
    for(const auto& file : input_files_)
    {
      nb_bit += 8 + file.name_.size() * 7 + 31 + 31;
    }

    return nb_bit / 8 + 1;
  }

  void Header::encode(std::vector<char>& out)
  {
    BitFileGenerator bit(31, 16, 8);
    bit.writeType3(description_file_.name_.size());
    for(const auto& c : description_file_.name_)
      bit.writeChar(c);
    bit.writeType1(description_file_.offset_);
    bit.writeType1(description_file_.size_);

    bit.writeType2(input_files_.size());
    for(const auto& file : input_files_)
    {
      bit.writeType3(file.name_.size());
      for(const auto& c : file.name_)
        bit.writeChar(c);
      bit.writeType1(file.offset_);
      bit.writeType1(file.size_);
    }

    std::vector<char> tmp = bit.get();
    out.insert(out.end(), tmp.begin(), tmp.end());
  }

  void Header::decode(const std::vector<char>& data)
  {
    BitFileGetter bit(31, 16, 8);
    bit.set(data);

    size_t file_size = bit.getType3();
    for(size_t i = 0; i < file_size; i++)
      description_file_.name_.push_back(bit.getChar());
    description_file_.offset_ = bit.getType1();
    description_file_.size_ = bit.getType1();

    size_t nb_file = bit.getType2();
    for(size_t j = 0; j < nb_file; j++)
    {
      File_t file;
      file_size = bit.getType3();
      for(size_t i = 0; i < file_size; i++)
        file.name_.push_back(bit.getChar());
      file.offset_ = bit.getType1();
      file.size_ = bit.getType1();
      input_files_.push_back(file);
    }
  }

} // namespace mementar
