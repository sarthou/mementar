#ifndef MEMENTAR_COMPRESSEDLEAFSESSION_H
#define MEMENTAR_COMPRESSEDLEAFSESSION_H

#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  class CompressedLeafSession
  {
  public:
    CompressedLeafSession(const time_t& key, size_t index);

    time_t getKey() const { return key_; }
    size_t getIndex() const { return index_; }

    BplusTree<time_t, Fact*>* getTree(Header& header, Archive& arch) const;
    std::vector<char> getRawData(Header& header, Archive& arch) const;

  private:
    time_t key_;
    size_t index_;
  };

} // namespace mementar

#endif // MEMENTAR_COMPRESSEDLEAFSESSION_H
