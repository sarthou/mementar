#ifndef MEMENTAR_HEADER_H
#define MEMENTAR_HEADER_H

#include <cstddef>
#include <string>
#include <vector>

namespace mementar {

  struct File_t
  {
    File_t(const std::string& name = "",
           size_t offset = 0,
           size_t size = 0) : name_(name),
                              path_(name)
    {
      while(name_.find('/') != std::string::npos)
        name_ = name_.substr(name_.find('/') + 1);
      offset_ = offset;
      size_ = size;
    }

    std::string name_;
    std::string path_;
    size_t offset_;
    size_t size_;
  };

  class Header
  {
  public:
    Header() = default;

    File_t description_file_;
    std::vector<File_t> input_files_;

    std::string toString();

    size_t encodedSize();
    void encode(std::vector<char>& out);
    void decode(const std::vector<char>& data);
  };

} // namespace mementar

#endif // MEMENTAR_HEADER_H
