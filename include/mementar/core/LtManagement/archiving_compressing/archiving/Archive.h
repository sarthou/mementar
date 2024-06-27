#ifndef MEMENTAR_ARCHIVE_H
#define MEMENTAR_ARCHIVE_H

#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"
#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BinaryManager.h"

namespace mementar {

  class Archive : public BinaryManager
  {
  public:
    Archive(const std::string& description, const std::vector<std::string>& files);
    Archive(const std::string& description, const Header& header);
    Archive();

    bool readBinaryFile(const std::string& file_name)
    {
      return BinaryManager::readBinaryFile(data_, file_name);
    }

    std::vector<char> load();
    std::vector<char> load(const std::vector<std::string>& raw_datas);

    Header getHeader(const std::vector<char>& data);
    Header getHeader() { return getHeader(data_); }

    std::string extractDescription(const Header& head, const std::vector<char>& data);
    std::string extractDescription(const Header& head)
    {
      return extractDescription(head, data_);
    }
    std::string extractFile(size_t index, const Header& head, const std::vector<char>& data);
    std::string extractFile(size_t index, const Header& head)
    {
      return extractFile(index, head, data_);
    }

  private:
    Header header_;
    std::string description_;
    std::vector<char> data_;
  };

} // namespace mementar

#endif // MEMENTAR_ARCHIVE_H
