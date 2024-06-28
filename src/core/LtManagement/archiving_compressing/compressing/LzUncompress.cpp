#include "mementar/core/LtManagement/archiving_compressing/compressing/LzUncompress.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BinaryManager.h"
#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGetter.h"

namespace mementar {

  LzUncompress::LzUncompress() : BinaryManager("mlz")
  {}

  std::string LzUncompress::uncompress(const std::vector<char>& data)
  {
    std::string out;
    BitFileGetter bit(0, 0, 8);
    bit.set(data);

    size_t offset = 0;

    const auto out_file_size = toInteger<size_t>({(uint8_t)bit.getType3(), (uint8_t)bit.getType3(), (uint8_t)bit.getType3(), (uint8_t)bit.getType3()});
    out.reserve(out_file_size);

    const size_t search_size = (size_t)toInteger<uint16_t>({(uint8_t)bit.getType3(), (uint8_t)bit.getType3()});
    bit.setSize1(neededBitCount(search_size));

    const size_t la_size = (size_t)toInteger<uint16_t>({(uint8_t)bit.getType3(), (uint8_t)bit.getType3()});
    bit.setSize2(neededBitCount(la_size));

    while(out.size() < out_file_size)
    {
      if(bit.getBit())
      {
        offset = bit.getType1();
        out += out.substr(out.size() - offset, bit.getType2());
      }
      else
        out.push_back(bit.getChar());
    }

    return out;
  }

  int LzUncompress::neededBitCount(size_t max_value)
  {
    int nb_bit = 1;
    size_t tmp_max = 2;
    while(tmp_max < max_value)
    {
      tmp_max = tmp_max << 1;
      nb_bit++;
    }
    return nb_bit;
  }

} // namespace mementar
