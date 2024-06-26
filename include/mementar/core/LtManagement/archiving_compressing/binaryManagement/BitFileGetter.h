#ifndef MEMENTAR_BITFILEGETTER_H
#define MEMENTAR_BITFILEGETTER_H

#include <array>
#include <cstring>
#include <string>
#include <vector>

namespace mementar {

  template<typename T>
  constexpr T toInteger(const std::array<unsigned char, sizeof(T)>& bytes) noexcept
  {
    T ret{};
    std::memcpy(&ret, bytes.data(), sizeof(T));
    return ret;
  }

  class BitFileGetter
  {
  public:
    BitFileGetter(uint32_t size_1 = 0, uint32_t size_2 = 0, uint32_t size_3 = 0, uint32_t size_4 = 0);

    void setSize1(uint32_t size_1) { type_1_size_ = size_1; }
    void setSize2(uint32_t size_2) { type_2_size_ = size_2; }
    void setSize3(uint32_t size_3) { type_3_size_ = size_3; }
    void setSize4(uint32_t size_4) { type_4_size_ = size_4; }

    uint32_t getType1();
    uint32_t getType2();
    uint32_t getType3();
    uint32_t getType4();
    char getChar();

    bool getBit();

    bool end(size_t offset = 0);

    void set(const std::vector<char>& data)
    {
      data_ = data;
      current_data_ = data_[0];
    }

  private:
    std::vector<char> data_;
    char current_data_;
    size_t major_index_;
    uint8_t minor_index_;

    uint32_t type_1_size_;
    uint32_t type_2_size_;
    uint32_t type_3_size_;
    uint32_t type_4_size_;
  };

} // namespace mementar

#endif // MEMENTAR_BITFILEGETTER_H
