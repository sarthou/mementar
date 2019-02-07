#include "mementor/lz/LzCompress.h"

#include <iostream>
#include <fstream>

LzCompress::LzCompress(size_t search_size, size_t la_size) : bit(bitConter(search_size), bitConter(la_size))
{
  // la_size_ <= search_size_
  search_size_ = search_size;
  la_size_ = la_size;
}

void LzCompress::compress(std::string& in, const std::string& out)
{
  size_t in_size = in.size();

  bit.writeChar(in_size >> 0);
  bit.writeChar(in_size >> 8);
  bit.writeChar(in_size >> 16);
  bit.writeChar(in_size >> 24);

  bit.writeBitFalse();
  bit.writeChar(in[0]);

  size_t cursor = 1;
  size_t index = -1;
  size_t length = 1;
  size_t tmp_length = 1;
  size_t i = -1;

  size_t min_size = (in_size < search_size_) ? in_size : search_size_;

  while (cursor < min_size)
  {
    length = 1;
    char c_tmp = in[cursor];
    for(i = 0; i < cursor; i++)
    {
      if(in[i] == c_tmp)
      {
        tmp_length = 1;
        while((in[i + tmp_length] == in[cursor + tmp_length]) && (tmp_length < la_size_ - 1) && (i + tmp_length < cursor))
          tmp_length++;

        if(tmp_length > length)
        {
          length = tmp_length;
          index = i;
        }
      }
    }

    if(length > 1)
    {
      bit.writeBitTrue();
      bit.writeType1(cursor - index);
      bit.writeType2(length);
    }
    else
    {
      bit.writeBitFalse();
      bit.writeChar(c_tmp);
    }

    cursor += length;
  }

  while (cursor < in_size)
  {
    length = 1;
    char c_tmp = in[cursor];
    for(i = cursor - search_size_ + 1; i < cursor; i++)
    {
      if(in[i] == c_tmp)
      {
        tmp_length = 1;
        while((in[i + tmp_length] == in[cursor + tmp_length]) && (tmp_length < la_size_ - 1) && (i + tmp_length < cursor))
          tmp_length++;

        if(tmp_length > length)
        {
          length = tmp_length;
          index = i;
        }
      }
    }

    if(length > 1)
    {
      bit.writeBitTrue();
      bit.writeType1(cursor - index);
      bit.writeType2(length);
    }
    else
    {
      bit.writeBitFalse();
      bit.writeChar(c_tmp);
    }

    cursor += length;
  }

  std::vector<char> out_vect = bit.get();

  std::cout << "Compression rate : " << (1 - (out_vect.size() / (float)in.size())) * 100.0f << std::endl;

  std::ofstream outfile;
	outfile.open(out + ".mlz", std::ios::binary | std::ios::out);
  std::string str(out_vect.begin(), out_vect.end());
	outfile.write(str.c_str(), str.length());
	outfile.close();

  std::cout << "Saved into " << out << ".mlz" << std::endl;
}

int LzCompress::bitConter(size_t max_value)
{
  int nb_bit = 1;
  int tmp_max = 2;
  while(tmp_max < max_value)
  {
    tmp_max = tmp_max<<1;
    nb_bit++;
  }
  return nb_bit;
}
