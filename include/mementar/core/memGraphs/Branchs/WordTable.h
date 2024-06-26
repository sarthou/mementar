#ifndef MEMENTAR_WORDTABLE_H
#define MEMENTAR_WORDTABLE_H

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace mementar {

  class WordTable
  {
  public:
    uint32_t add(const std::string& value)
    {
      table_.push_back(value);
      return table_.size() - 1;
    }

    std::string& get(uint32_t index)
    {
      return table_[index];
    }

    std::string& operator[](uint32_t index)
    {
      return table_[index];
    }

    const std::string& operator[](uint32_t index) const
    {
      return table_[index];
    }

    void index2string(std::unordered_set<std::string>& res, const std::unordered_set<uint32_t>& base)
    {
      for(uint32_t i : base)
        res.insert(table_[i]);
    }

    void reset()
    {
      table_.clear();
    }

  private:
    std::vector<std::string> table_;
  };

} // namespace mementar

#endif // MEMENTAR_WORDTABLE_H
