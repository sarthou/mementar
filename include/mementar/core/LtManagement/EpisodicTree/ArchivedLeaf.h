#ifndef MEMENTAR_ARCHIVEDLEAF_H
#define MEMENTAR_ARCHIVEDLEAF_H

#include <ctime>
#include <string>

#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafNode.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"

namespace mementar {

  class ArchivedLeaf
  {
  public:
    ArchivedLeaf(CompressedLeafNode* tree, size_t nb, const std::string& directory);
    ArchivedLeaf(const SoftPoint::Ttime& key, const std::string& directory);

    std::string getDirectory() const { return directory_; }
    SoftPoint::Ttime getKey() const { return key_; }

    BplusTree<SoftPoint::Ttime, Fact*>* getTree(size_t i);
    std::vector<Context> getContexts();

  private:
    SoftPoint::Ttime key_;
    std::string directory_;
  };

} // namespace mementar

#endif // MEMENTAR_ARCHIVEDLEAF_H
