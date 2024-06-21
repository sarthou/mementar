#ifndef MEMENTAR_HUFFMAN_H
#define MEMENTAR_HUFFMAN_H

#include <array>
#include <limits>
#include <string>

#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BinaryManager.h"

namespace mementar {

  struct HuffCode_t
  {
    uint32_t value_;
    uint8_t size_;
  };

  struct HuffNode_t
  {
    using Index = std::uint16_t;
    using Frequency = std::uint32_t;

    static constexpr Index INVALID_INDEX = std::numeric_limits<Index>::max();

    Frequency freq{};
    Index left{INVALID_INDEX};
    Index right{INVALID_INDEX};
    HuffCode_t code;
  };

  // Number of values representable with a byte
  static constexpr HuffNode_t::Index VALUE_IN_BYTE = 1 << (sizeof(std::uint8_t) * 8);

  // Maximum number of leaf nodes (character) in an Huffman tree
  static constexpr HuffNode_t::Index LEAF_COUNT = VALUE_IN_BYTE;

  // Maximum number of nodes (leaf + bind) in an Huffman tree
  static constexpr HuffNode_t::Index NODE_COUNT = LEAF_COUNT * 2;

  using FrequencyMap = std::array<HuffNode_t::Frequency, LEAF_COUNT>;
  using NodeList = std::array<HuffNode_t, NODE_COUNT>;
  using TreeList = std::array<size_t, VALUE_IN_BYTE>;

  class Huffman : public BinaryManager
  {
  public:
    Huffman();
    void analyse(const std::string& data, std::size_t jobs = 1);
    void generateCode();
    std::vector<char> getTreeCode();
    std::vector<char> getDataCode(const std::string& in_data);

    size_t setTree(const std::vector<char>& in);
    std::string getFile(const std::vector<char>& data);

  private:
    FrequencyMap frequencies_{};
    NodeList nodes_{};
    TreeList subtrees_{};
    size_t h_min_;

    void sum(const FrequencyMap& other, FrequencyMap& into);
    void countChar(const std::string& text, std::size_t jobs = 1);
    HuffNode_t::Index generateTree();
    void generateCode(HuffNode_t::Index index);
  };

} // namespace mementar

#endif // MEMENTAR_HUFFMAN_H
