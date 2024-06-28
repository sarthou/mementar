#ifndef MEMENTAR_COMPRESSEDLEAFSESSION_H
#define MEMENTAR_COMPRESSEDLEAFSESSION_H

#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"

namespace mementar {

  class CompressedLeafSession
  {
  public:
    CompressedLeafSession(const SoftPoint::Ttime& key, size_t index);

    SoftPoint::Ttime getKey() const { return key_; }
    size_t getIndex() const { return index_; }

    BplusTree<SoftPoint::Ttime, Fact*>* getTree(const Header& header, Archive& arch) const;
    std::vector<char> getRawData(const Header& header, Archive& arch) const;

  private:
    SoftPoint::Ttime key_;
    size_t index_;
  };

} // namespace mementar

#endif // MEMENTAR_COMPRESSEDLEAFSESSION_H
