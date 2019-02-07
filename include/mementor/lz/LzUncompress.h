#ifndef LZUNCOMPRESS_H
#define LZUNCOMPRESS_H

#include "mementor/lz/BitFileGetter.h"

class LzUncompress
{
public:
  LzUncompress(size_t search_size = 512, size_t la_size = 32);

  void uncompress(const std::string& in, std::string& out);

private:
  size_t search_size_;
  size_t la_size_;

  BitFileGetter bit;

  int bitConter(size_t max_value);
};

#endif // LZUNCOMPRESS_H