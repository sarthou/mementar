#include "mementar/core/LtManagement/archiving_compressing/compressing/Huffman_old.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BinaryManager.h"
#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGenerator.h"
#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGetter.h"

namespace mementar {

#define TREE_CHAR_SIZE 8
#define TREE_VALUE_SIZE 6
#define TREE_VALUE_SIZE_SIZE 31 // do not go over 31

  void printTree(HuffNode_t* node)
  {
    for(size_t i = 0; i < node->code_.size_; i++)
      std::cout << " ";
    std::cout << std::hex << node->code_.value_ << std::endl;

    if(node->left_ != nullptr)
      printTree(node->left_);
    if(node->right_ != nullptr)
      printTree(node->right_);
  }

  bool comparePtrNode(HuffNode_t* a, HuffNode_t* b)
  {
    return (a->freq_ < b->freq_);
  }

  bool comparePtrNodeByCode(HuffNode_t* a, HuffNode_t* b)
  {
    if(a->code_.size_ < b->code_.size_)
      return false;
    else if(a->code_.size_ > b->code_.size_)
      return true;
    else if(a->code_.value_ < b->code_.value_)
      return true;
    else
      return false;
  }

  HuffmanOld::HuffmanOld() : BinaryManager("mhu")
  {
  }

  HuffmanOld::~HuffmanOld()
  {
    for(auto* it : heap_)
      delete it;
  }

  void HuffmanOld::analyse(std::vector<char>& data)
  {
    for(const auto& c : data)
    {
      auto it = leaf_map_.find(c);
      if(it == leaf_map_.end())
      {
        HuffNode_t* leaf = new HuffNode_t(c);
        heap_.push_back(leaf);
        leaf_map_[c] = leaf;
      }
      else
        it->second->freq_++;
    }
  }

  void HuffmanOld::generateTree()
  {
    while(heap_.size() > 1)
    {
      std::sort(heap_.begin(), heap_.end(), comparePtrNode);

      HuffNode_t* tmp = new HuffNode_t();
      tmp->right_ = heap_[0];
      tmp->left_ = heap_[1];
      tmp->freq_ = tmp->right_->freq_ + tmp->left_->freq_;
      heap_.erase(heap_.begin());
      heap_.erase(heap_.begin());
      heap_.push_back(tmp);
    }

    heap_[0]->code_.value_ = 0;
    heap_[0]->code_.size_ = 0;
    generateCode(heap_[0]);
  }

  void HuffmanOld::generateCode(HuffNode_t* node)
  {
    if(node->right_ != nullptr)
    {
      node->right_->code_.value_ = (node->code_.value_ << 1);
      node->right_->code_.size_ = node->code_.size_ + 1;
      generateCode(node->right_);
    }
    if(node->left_ != nullptr)
    {
      node->left_->code_.value_ = (node->code_.value_ << 1) | 0x01;
      node->left_->code_.size_ = node->code_.size_ + 1;
      generateCode(node->left_);
    }
  }

  void HuffmanOld::getTreeCode(std::vector<char>& out)
  {
    BitFileGenerator bit(TREE_CHAR_SIZE, TREE_VALUE_SIZE, TREE_VALUE_SIZE_SIZE);
    // coding tree
    bit.writeType1((leaf_map_.size() >> 0) & 0x000000ff);
    bit.writeType1((leaf_map_.size() >> 8) & 0x000000ff);

    for(const auto& it : leaf_map_)
    {
      bit.writeType1(it.second->data_);
      bit.writeType2(it.second->code_.size_);
      bit.writeType3(it.second->code_.value_);
    }
    std::vector<char> tmp;
    bit.get(tmp);
    out.insert(out.end(), tmp.begin(), tmp.end());
  }

  void HuffmanOld::getDataCode(std::vector<char>& data, std::vector<char>& out)
  {
    BitFileGenerator bit(8);

    bit.writeType1((data.size() >> 0) & 0x000000ff);
    bit.writeType1((data.size() >> 8) & 0x000000ff);
    bit.writeType1((data.size() >> 16) & 0x000000ff);
    bit.writeType1((data.size() >> 24) & 0x000000ff);

    for(auto c : data)
    {
      auto& code = leaf_map_.find(c)->second->code_;
      for(uint32_t i = 1 << (code.size_ - 1); i > 0; i >>= 1)
        if((code.value_ & i) != 0)
          bit.writeBitTrue();
        else
          bit.writeBitFalse();
    }
    std::vector<char> tmp = bit.get();
    out.insert(out.end(), tmp.begin(), tmp.end());
  }

  size_t HuffmanOld::setTree(const std::vector<char>& in)
  {
    BitFileGetter bit(TREE_CHAR_SIZE, TREE_VALUE_SIZE, TREE_VALUE_SIZE_SIZE);
    bit.set(in);

    auto nb_leaf = toInteger<uint16_t>({(uint8_t)bit.getType1(), (uint8_t)bit.getType1()});

    for(size_t i = 0; i < nb_leaf; i++)
    {
      HuffNode_t* tmp_node = new HuffNode_t(bit.getType1());
      tmp_node->code_.size_ = bit.getType2();
      tmp_node->code_.value_ = bit.getType3();
      heap_.push_back(tmp_node);
    }

    std::sort(heap_.begin(), heap_.end(), comparePtrNodeByCode);
    while(heap_.size() > 1)
    {
      HuffNode_t* tmp_node = new HuffNode_t;
      tmp_node->left_ = heap_[1];
      tmp_node->right_ = heap_[0];
      tmp_node->code_.size_ = heap_[0]->code_.size_ - 1;
      tmp_node->code_.value_ = (heap_[0]->code_.value_ >> 1) & 0xefffffff;
      heap_.erase(heap_.begin());
      heap_.erase(heap_.begin());
      heap_.push_back(tmp_node);
      std::sort(heap_.begin(), heap_.end(), comparePtrNodeByCode);
    }

    size_t tree_bit_size = (2 * 8 + nb_leaf * (TREE_CHAR_SIZE + TREE_VALUE_SIZE + TREE_VALUE_SIZE_SIZE));

    return tree_bit_size / 8 + 1;
  }

  void HuffmanOld::getFile(const std::vector<char>& data, std::string& out)
  {
    out = "";
    BitFileGetter bit(8);
    bit.set(data);

    auto out_file_size = toInteger<size_t>({(uint8_t)bit.getType1(), (uint8_t)bit.getType1(), (uint8_t)bit.getType1(), (uint8_t)bit.getType1()});
    out.reserve(out_file_size);
    while(out.size() < out_file_size)
    {
      HuffNode_t* node = heap_[0];
      while(node->right_ != nullptr)
      {
        if(bit.getBit())
          node = node->left_;
        else
          node = node->right_;
      }
      out += node->data_;
    }
  }

} // namespace mementar
