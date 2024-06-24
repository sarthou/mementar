#ifndef MEMENTAR_HUFFMAN_OLD_H
#define MEMENTAR_HUFFMAN_OLD_H

#include <map>
#include <set>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BinaryManager.h"

namespace mementar {

  struct HuffCode_t
  {
    uint32_t value_;
    uint8_t size_;

    HuffCode_t() : value_(0), size_(0) {}
  };

  struct HuffNode_t
  {
    size_t freq_;
    char data_;
    HuffCode_t code_;
    HuffNode_t* left_;
    HuffNode_t* right_;

    HuffNode_t() : freq_(1),
                   data_(' '),
                   left_(nullptr),
                   right_(nullptr)
    {}

    explicit HuffNode_t(char data) : freq_(1),
                                     data_(data),
                                     left_(nullptr),
                                     right_(nullptr)
    {}

    ~HuffNode_t()
    {
      delete left_;
      delete right_;
    }
  };

  class HuffmanOld : public BinaryManager
  {
  public:
    void analyse(std::vector<char>& data);
    void generateTree();
    void getTreeCode(std::vector<char>& out);
    void getDataCode(std::vector<char>& data, std::vector<char>& out);

    size_t setTree(const std::vector<char>& in);
    void getFile(const std::vector<char>& data, std::string& out);

    HuffmanOld();
    ~HuffmanOld();

  private:
    std::vector<HuffNode_t*> heap_;
    std::map<char, HuffNode_t*> leaf_map_;

    void generateCode(HuffNode_t* node);
  };

} // namespace mementar

#endif // MEMENTAR_HUFFMAN_OLD_H
