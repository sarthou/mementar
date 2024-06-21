#include "mementar/core/LtManagement/archiving_compressing/compressing/Huffman.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <future>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGenerator.h"
#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGetter.h"

#define TREE_CHAR_SIZE 8
#define TREE_VALUE_SIZE 6
#define TREE_VALUE_SIZE_SIZE 31 // do not go over 31

template<typename T, std::size_t N>
struct static_alloc // NOLINT
{
  using value_type = T;
  static_alloc() = default;
  template<typename U>
  constexpr static_alloc(const static_alloc<U, N>&) noexcept {};
  [[nodiscard]] constexpr value_type* allocate(std::size_t n)
  {
    return buffer.data();
  }
  constexpr void deallocate(value_type* p, std::size_t n) {}
  template<typename U>
  struct rebind // NOLINT
  {
    using other = static_alloc<U, N>;
  };
  std::array<value_type, N> buffer{};
};
template<typename T, typename U, std::size_t N>
bool operator==(const static_alloc<T, N>&, const static_alloc<U, N>&)
{
  return true;
}
template<typename T, typename U, std::size_t N>
bool operator!=(const static_alloc<T, N>&, const static_alloc<U, N>&)
{
  return false;
}

namespace mementar {

  class NodeCompare
  {
  public:
    explicit NodeCompare(const NodeList& nodes) : nodes_{nodes} {}
    bool operator()(HuffNode_t::Index l, HuffNode_t::Index r)
    {
      return nodes_[l].freq > nodes_[r].freq;
    }

  private:
    const NodeList& nodes_;
  };

  class NodeValueCompare
  {
  public:
    explicit NodeValueCompare(const NodeList& nodes) : nodes_{nodes} {}
    bool operator()(HuffNode_t::Index l, HuffNode_t::Index r)
    {
      if(nodes_[l].code.size_ < nodes_[r].code.size_)
        return true;
      else if(nodes_[l].code.size_ > nodes_[r].code.size_)
        return false;
      else if(nodes_[l].code.value_ < nodes_[r].code.value_)
        return false;
      else
        return true;
    }

  private:
    const NodeList& nodes_;
  };

  Huffman::Huffman() : BinaryManager("mhu")
  {
  }

  void Huffman::analyse(const std::string& data, std::size_t jobs)
  {
    countChar(data, jobs);
  }

  void Huffman::generateCode()
  {
    auto root_node = generateTree();
    std::cout << "generateTree" << std::endl;
    nodes_[root_node].code.size_ = 0;
    nodes_[root_node].code.value_ = 0;
    generateCode(root_node);
  }

  std::vector<char> Huffman::getTreeCode()
  {
    BitFileGenerator bit(TREE_CHAR_SIZE, TREE_VALUE_SIZE, TREE_VALUE_SIZE_SIZE);
    size_t valid_leaf = 0;
    for(size_t i = 0; i < LEAF_COUNT; i++)
      if(nodes_[i].freq != 0)
        valid_leaf++;

    // coding tree
    bit.writeType1((valid_leaf >> 0) & 0x000000ff);
    bit.writeType1((valid_leaf >> 8) & 0x000000ff);

    for(size_t i = 0; i < LEAF_COUNT; i++)
    {
      if(nodes_[i].freq != 0)
      {
        bit.writeType1((char)i);
        bit.writeType2(nodes_[i].code.size_);
        bit.writeType3(nodes_[i].code.value_);
      }
    }

    return bit.get();
  }

  std::vector<char> Huffman::getDataCode(const std::string& in_data)
  {
    BitFileGenerator bit(8);
    bit.resize(in_data.size());

    bit.writeType1((in_data.size() >> 0) & 0x000000ff);
    bit.writeType1((in_data.size() >> 8) & 0x000000ff);
    bit.writeType1((in_data.size() >> 16) & 0x000000ff);
    bit.writeType1((in_data.size() >> 24) & 0x000000ff);

    for(const std::uint8_t& c : in_data)
      bit.writeNReverse(nodes_[c].code.size_, nodes_[c].code.value_);

    return bit.get();
  }

  size_t Huffman::setTree(const std::vector<char>& in)
  {
    BitFileGetter bit(TREE_CHAR_SIZE, TREE_VALUE_SIZE, TREE_VALUE_SIZE_SIZE);
    bit.set(in);

    auto nb_leaf = toInteger<uint16_t>({(uint8_t)bit.getType1(), (uint8_t)bit.getType1()});

    using minheap = std::priority_queue<
      HuffNode_t::Index,
      std::vector<HuffNode_t::Index, static_alloc<HuffNode_t::Index, LEAF_COUNT>>,
      NodeValueCompare>;

    minheap heap{NodeValueCompare{nodes_}};
    h_min_ = LEAF_COUNT;

    for(size_t i = 0; i < nb_leaf; i++)
    {
      HuffNode_t::Index data = bit.getType1();
      nodes_[data].code.size_ = bit.getType2();
      nodes_[data].code.value_ = bit.getType3();
      heap.push(data);
      if(nodes_[data].code.size_ < h_min_)
        h_min_ = nodes_[data].code.size_;
    }

    HuffNode_t::Index bind_node_index = LEAF_COUNT; // bind nodes are stored after all leaves

    while(heap.size() != 1)
    {
      auto right = heap.top();
      heap.pop();
      auto left = heap.top();
      heap.pop();

      nodes_[bind_node_index].right = right;
      nodes_[bind_node_index].left = left;
      nodes_[bind_node_index].code.size_ = nodes_[right].code.size_ - 1;
      nodes_[bind_node_index].code.value_ = (nodes_[right].code.value_ >> 1) & 0xefffffff;
      if(nodes_[bind_node_index].code.size_ == h_min_)
        subtrees_[nodes_[bind_node_index].code.value_] = bind_node_index;

      heap.push(bind_node_index);
      ++bind_node_index;
    }

    for(size_t i = 0; i < LEAF_COUNT; i++)
    {
      if(nodes_[i].code.size_ == h_min_)
        subtrees_[nodes_[i].code.value_] = i;
    }

    size_t tree_bit_size = (2 * 8 + nb_leaf * (TREE_CHAR_SIZE + TREE_VALUE_SIZE + TREE_VALUE_SIZE_SIZE));

    return tree_bit_size / 8 + 1;
  }

  std::string Huffman::getFile(const std::vector<char>& data)
  {
    std::string out;

    const auto out_file_size = toInteger<size_t>({(uint8_t)data[0], (uint8_t)data[1], (uint8_t)data[2], (uint8_t)data[3]});
    out.reserve(out_file_size);

    const auto nodes = nodes_.data();
    auto node = &nodes_[0];

    uint8_t mask_index = 8;
    uint8_t bit_data = 0x00;
    uint8_t h_min_data = 0x00;

    uint32_t index = 3;

    for(;;)
    {
      h_min_data = 0x00;
      for(size_t i = 0; i < h_min_; i++)
      {
        bit_data >>= 1;
        if((++mask_index) >= 8)
        {
          bit_data = data[++index];
          mask_index = 0;
        }
        h_min_data = (h_min_data << 1) | (bit_data & 0x01);
      }
      node = nodes + subtrees_[h_min_data];

      while(node->right - HuffNode_t::INVALID_INDEX)
      {
        bit_data >>= 1;
        if((++mask_index) >= 8)
        {
          bit_data = data[++index];
          mask_index = 0;
        }
        node = ((bit_data & 0x01) ? nodes + node->left : nodes + node->right);
      }
      out += (char)(node - nodes);
      if(out.size() == out_file_size)
        break;
    }

    return out;
  }

  void Huffman::sum(const FrequencyMap& other, FrequencyMap& into)
  {
    std::transform(std::begin(other), std::end(other), std::begin(into),
                   std::begin(into), std::plus<>{});
  }

  FrequencyMap count_char_impl(const std::string& text)
  {
    FrequencyMap freq{}; // Zero-initialized array
    for(std::uint8_t c : text)
      freq[c]++;
    return freq;
  }

  void Huffman::countChar(const std::string& text, std::size_t jobs)
  {
    if(jobs <= 1)
    {
      sum(count_char_impl(text), frequencies_);
      return;
    }

    const auto thread_count = jobs - 1;
    const auto bound = text.size() / thread_count;

    std::vector<std::future<FrequencyMap>> counting_units(thread_count);

    size_t lower_bound = 0;
    for(auto& unit : counting_units)
    {
      unit = std::async(std::launch::async, count_char_impl,
                        text.substr(lower_bound, bound));
      lower_bound += bound;
    }
    sum(count_char_impl(text.substr(lower_bound)), frequencies_);

    for(auto& unit : counting_units)
      sum(unit.get(), frequencies_);
  }

  HuffNode_t::Index Huffman::generateTree()
  {
    using minheap = std::priority_queue<
      HuffNode_t::Index,
      std::vector<HuffNode_t::Index, static_alloc<HuffNode_t::Index, LEAF_COUNT>>,
      NodeCompare>;

    minheap heap{NodeCompare{nodes_}};

    for(HuffNode_t::Index i = 0; i < frequencies_.size(); ++i)
    {
      if(frequencies_[i] == 0)
        continue; // Do not set unused characters
      nodes_[i].freq = frequencies_[i];
      heap.push(i);
    }

    HuffNode_t::Index bind_node_index = LEAF_COUNT; // bind nodes are stored after all leaves

    while(heap.size() != 1)
    {
      auto right = heap.top();
      heap.pop();
      auto left = heap.top();
      heap.pop();

      nodes_[bind_node_index].freq =
        nodes_[left].freq + nodes_[right].freq;
      nodes_[bind_node_index].left = left;
      nodes_[bind_node_index].right = right;
      heap.push(bind_node_index);

      ++bind_node_index;
    }
    return heap.top();
  }

  void Huffman::generateCode(HuffNode_t::Index index)
  {
    if(nodes_[index].right != HuffNode_t::INVALID_INDEX)
    {
      nodes_[nodes_[index].right].code.value_ = (nodes_[index].code.value_ << 1);
      nodes_[nodes_[index].right].code.size_ = nodes_[index].code.size_ + 1;
      generateCode(nodes_[index].right);
    }
    if(nodes_[index].left != HuffNode_t::INVALID_INDEX)
    {
      nodes_[nodes_[index].left].code.value_ = (nodes_[index].code.value_ << 1) | 0x01;
      nodes_[nodes_[index].left].code.size_ = nodes_[index].code.size_ + 1;
      generateCode(nodes_[index].left);
    }
  }
} // namespace mementar
