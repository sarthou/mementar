#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGetter.h"

#include <cstddef>
#include <cstdint>

namespace mementar {

  BitFileGetter::BitFileGetter(uint32_t size_1, uint32_t size_2, uint32_t size_3, uint32_t size_4) : current_data_(0),
                                                                                                     major_index_(0),
                                                                                                     minor_index_(0),
                                                                                                     type_1_size_(size_1),
                                                                                                     type_2_size_(size_2),
                                                                                                     type_3_size_(size_3),
                                                                                                     type_4_size_(size_4)
  {}

  uint32_t BitFileGetter::getType1()
  {
    uint32_t res = 0;
    int8_t to_get = (int8_t)type_1_size_;
    for(;;)
    {
      // const uint32_t working_data = (current_data_ & 0x000000ff) >> minor_index_;
      // res |= ((working_data & ~((0xffffffff) << to_get)) << (type_1_size_ - to_get));
      res |= ((((current_data_ & 0x000000ff) >> minor_index_) &
               ~((0xffffffff) << to_get))
              << (type_1_size_ - to_get));

      const uint8_t getted = (minor_index_ + to_get > 7) ? 8 - minor_index_ : to_get;

      to_get -= getted;
      if(to_get > 0)
      {
        current_data_ = data_[++major_index_];
        minor_index_ = 0;
      }
      else
      {
        minor_index_ += getted;
        if(minor_index_ > 7)
        {
          minor_index_ = 0;
          current_data_ = data_[++major_index_];
        }
        return res;
      }
    }
  }

  uint32_t BitFileGetter::getType2()
  {
    uint32_t res = 0;
    int8_t to_get = (int8_t)type_2_size_;
    for(;;)
    {
      // const uint32_t working_data = (current_data_ & 0x000000ff) >> minor_index_;
      // res |= ((working_data & ~((0xffffffff) << to_get)) << (type_1_size_ - to_get));
      res |= ((((current_data_ & 0x000000ff) >> minor_index_) &
               ~((0xffffffff) << to_get))
              << (type_2_size_ - to_get));

      const uint8_t getted = (minor_index_ + to_get > 7) ? 8 - minor_index_ : to_get;

      to_get -= getted;
      if(to_get > 0)
      {
        current_data_ = data_[++major_index_];
        minor_index_ = 0;
      }
      else
      {
        minor_index_ += getted;
        if(minor_index_ > 7)
        {
          minor_index_ = 0;
          current_data_ = data_[++major_index_];
        }
        return res;
      }
    }
  }

  uint32_t BitFileGetter::getType3()
  {
    uint32_t res = 0;
    int8_t to_get = (int8_t)type_3_size_;
    for(;;)
    {
      // const uint32_t working_data = (current_data_ & 0x000000ff) >> minor_index_;
      // res |= ((working_data & ~((0xffffffff) << to_get)) << (type_1_size_ - to_get));
      res |= ((((current_data_ & 0x000000ff) >> minor_index_) &
               ~((0xffffffff) << to_get))
              << (type_3_size_ - to_get));

      const uint8_t getted = (minor_index_ + to_get > 7) ? 8 - minor_index_ : to_get;

      to_get -= getted;
      if(to_get > 0)
      {
        current_data_ = data_[++major_index_];
        minor_index_ = 0;
      }
      else
      {
        minor_index_ += getted;
        if(minor_index_ > 7)
        {
          minor_index_ = 0;
          current_data_ = data_[++major_index_];
        }
        return res;
      }
    }
  }

  uint32_t BitFileGetter::getType4()
  {
    uint32_t res = 0;
    int8_t to_get = (int8_t)type_4_size_;
    for(;;)
    {
      // const uint32_t working_data = (current_data_ & 0x000000ff) >> minor_index_;
      // res |= ((working_data & ~((0xffffffff) << to_get)) << (type_1_size_ - to_get));
      res |= ((((current_data_ & 0x000000ff) >> minor_index_) &
               ~((0xffffffff) << to_get))
              << (type_4_size_ - to_get));

      const uint8_t getted = (minor_index_ + to_get > 7) ? 8 - minor_index_ : to_get;

      to_get -= getted;
      if(to_get > 0)
      {
        current_data_ = data_[++major_index_];
        minor_index_ = 0;
      }
      else
      {
        minor_index_ += getted;
        if(minor_index_ > 7)
        {
          minor_index_ = 0;
          current_data_ = data_[++major_index_];
        }
        return res;
      }
    }
  }

  char BitFileGetter::getChar()
  {
    char res = 0;
    int8_t to_get = 7;
    for(;;)
    {
      // const uint32_t working_data = (current_data_ & 0x000000ff) >> minor_index_;
      // res |= ((working_data & ~((0xffffffff) << to_get)) << (type_1_size_ - to_get));
      res |= ((((current_data_ & 0x000000ff) >> minor_index_) &
               ~((0xffffffff) << to_get))
              << (7 - to_get));

      const uint8_t getted = (minor_index_ + to_get > 7) ? 8 - minor_index_ : to_get;

      to_get -= getted;
      if(to_get > 0)
      {
        current_data_ = data_[++major_index_];
        minor_index_ = 0;
      }
      else
      {
        minor_index_ += getted;
        if(minor_index_ > 7)
        {
          minor_index_ = 0;
          current_data_ = data_[++major_index_];
        }
        return res;
      }
    }
  }

  bool BitFileGetter::getBit()
  {
    bool res = (current_data_ >> minor_index_) & 0x01;

    if(minor_index_ > 6)
    {
      minor_index_ = 0;
      current_data_ = data_[++major_index_];
    }
    else
      minor_index_++;

    return res;
  }

  bool BitFileGetter::end(size_t offset)
  {
    if(major_index_ >= data_.size())
    {
      if(minor_index_ + offset >= 8)
        return true;
      else
        return false;
    }
    else
      return false;
  }

} // namespace mementar
