#ifndef MEMENTAR_COMPRESSEDLEAF_H
#define MEMENTAR_COMPRESSEDLEAF_H

#include <ctime>
#include <string>

#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  class CompressedLeaf
  {
  public:
    CompressedLeaf(BplusTree<time_t, Fact*>* tree, const std::string& directory);
    CompressedLeaf(const time_t& key, const std::string& directory);

    std::string getDirectory() const { return directory_; }
    time_t getKey() const { return key_; }

    BplusTree<time_t, Fact*>* getTree();

  private:
    time_t key_;
    std::string directory_;

    std::string treeToString(BplusTree<time_t, Fact*>* tree);
  };

} // namespace mementar

#endif // MEMENTAR_COMPRESSEDLEAF_H
