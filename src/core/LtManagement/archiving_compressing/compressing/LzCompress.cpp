#include "mementar/core/LtManagement/archiving_compressing/compressing/LzCompress.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BinaryManager.h"

namespace mementar {

  LzCompress::LzCompress(size_t search_size, size_t la_size) : BinaryManager("mlz"),
                                                               search_size_(search_size),
                                                               search_size_1_(search_size_ - 1),
                                                               la_size_(la_size),
                                                               la_size_1_(la_size - 1),
                                                               bit_(neededBitCount(search_size), neededBitCount(la_size), 8)
  {
    // la_size_ <= search_size_
  }

  std::vector<char> LzCompress::compress(const std::string& in)
  {
    const size_t in_size = in.size();
    bit_.resize(in_size);

    bit_.writeType3(in_size >> 0);
    bit_.writeType3(in_size >> 8);
    bit_.writeType3(in_size >> 16);
    bit_.writeType3(in_size >> 24);

    bit_.writeType3(search_size_ >> 0);
    bit_.writeType3(search_size_ >> 8);

    bit_.writeType3(la_size_ >> 0);
    bit_.writeType3(la_size_ >> 8);

    bit_.writeBitFalse();
    bit_.writeChar(in[0]);

    size_t cursor = 1;
    size_t cursor_1 = 0;
    size_t index = -1;
    size_t length = 1;
    size_t tmp_length = 1;
    size_t search_index = 1;
    size_t la_index = 1;
    size_t i = -1;
    char c_tmp = 0;

    while(cursor < in_size)
    {
      length = 1;
      c_tmp = in[cursor];
      cursor_1 = cursor - 1;
      for(i = std::max(cursor - search_size_1_, size_t{0}); i < cursor_1; i++)
      {
        if(in[i] == c_tmp)
        {
          if(in[i + 1] == in[cursor + 1]) // directly check the second character in order
                                          // to avoid to do usless expensive tests
          {
            tmp_length = 1;
            search_index = i + 1;
            la_index = cursor + 1;
            while((in[search_index] == in[la_index]) && (tmp_length - la_size_1_ != 0) && (search_index < cursor))
            {
              ++tmp_length;
              ++search_index;
              ++la_index;
            }

            if(tmp_length > length)
            {
              length = tmp_length;
              index = i;
            }
          }
        }
      }

      if(length > 1)
      {
        bit_.writeBitTrue();
        bit_.writeType1(cursor - index);
        bit_.writeType2(length);
        cursor += length;
      }
      else
      {
        bit_.writeBitFalse();
        bit_.writeChar(c_tmp);
        ++cursor;
      }
    }

    return bit_.get();
  }

  int LzCompress::neededBitCount(size_t max_value)
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
