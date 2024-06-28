#ifndef MEMENTAR_COMPRESSEDLEAF_H
#define MEMENTAR_COMPRESSEDLEAF_H

#include <ctime>
#include <string>

#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"

namespace mementar {

  class CompressedLeaf
  {
  public:
    CompressedLeaf(BplusTree<SoftPoint::Ttime, Fact*>* tree, const std::string& directory);
    CompressedLeaf(const SoftPoint::Ttime& key, const std::string& directory);

    std::string getDirectory() const { return directory_; }
    SoftPoint::Ttime getKey() const { return key_; }

    BplusTree<SoftPoint::Ttime, Fact*>* getTree();

  private:
    SoftPoint::Ttime key_;
    std::string directory_;

    std::string treeToString(BplusTree<SoftPoint::Ttime, Fact*>* tree);
  };

} // namespace mementar

#endif // MEMENTAR_COMPRESSEDLEAF_H
