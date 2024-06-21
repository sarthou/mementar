#ifndef MEMENTAR_BITFILEGENERATOR_H
#define MEMENTAR_BITFILEGENERATOR_H

#include <string>
#include <vector>

namespace mementar {

  class BitFileGenerator
  {
  public:
    BitFileGenerator(uint8_t size_1 = 0, uint8_t size_2 = 0, uint8_t size_3 = 0, uint8_t size_4 = 0);
    void resize(size_t size) { data_.resize(size, 0); };

    void writeType1(uint32_t value);
    void writeType2(uint32_t value);
    void writeType3(uint32_t value);
    void writeType4(uint32_t value);
    void writeN(size_t size, uint32_t value);
    void writeNReverse(size_t size, uint32_t value);
    void writeChar(char value);

    void writeBitTrue();
    void writeBitFalse();

    std::vector<char> get() { return std::vector<char>(data_.begin(), data_.begin() + (long)major_index_ + 1); }
    void get(std::vector<char>& out) { out = data_; }

  private:
    std::vector<char> data_;
    size_t major_index_;
    uint8_t minor_index_;

    uint8_t type_1_size_;
    uint8_t type_2_size_;
    uint8_t type_3_size_;
    uint8_t type_4_size_;
  };

} // namespace mementar

#endif // MEMENTAR_BITFILEGENERATOR_H
