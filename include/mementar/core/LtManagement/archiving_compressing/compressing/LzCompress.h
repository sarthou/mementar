#ifndef MEMENTAR_LZCOMPRESS_H
#define MEMENTAR_LZCOMPRESS_H

#include <cstddef>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BinaryManager.h"
#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGenerator.h"

namespace mementar {

  class LzCompress : public BinaryManager
  {
  public:
    LzCompress(size_t search_size = 2048, size_t la_size = 64);

    std::vector<char> compress(const std::string& in);

  private:
    size_t search_size_;
    size_t search_size_1_;
    size_t la_size_;
    size_t la_size_1_;

    BitFileGenerator bit_;

    int neededBitCount(size_t max_value);
  };

} // namespace mementar

#endif // MEMENTAR_LZCOMPRESS_H
