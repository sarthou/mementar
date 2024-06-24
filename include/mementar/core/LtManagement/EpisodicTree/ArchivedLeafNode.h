#ifndef MEMENTAR_ARCHIVEDLEAFNODE_H
#define MEMENTAR_ARCHIVEDLEAFNODE_H

#include <atomic>
#include <cstddef>
#include <ctime>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "mementar/core/LtManagement/EpisodicTree/ArchivedLeaf.h"
#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafNode.h"
#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafNodeSession.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  class ArchivedLeafNode
  {
    using LeafType = typename BplusLeaf<SoftPoint::Ttime, Fact*>::LeafType;

  public:
    ArchivedLeafNode(const std::string& directory, size_t order = 10);
    ~ArchivedLeafNode();

    void insert(Fact* data);
    void remove(Fact* data);
    LeafType* find(SoftPoint::Ttime key);
    LeafType* findNear(SoftPoint::Ttime key);
    LeafType* getFirst();
    LeafType* getLast();

    void display(SoftPoint::Ttime key);

    void newSession();

  private:
    std::string directory_;
    size_t order_;
    mutable std::shared_timed_mutex mut_;

    // keys_.size() == btree_childs_.size() + compressed_childs_.size()
    // keys_[i] correspond to the first key of child i
    std::vector<SoftPoint::Ttime> keys_;
    std::vector<CompressedLeafNode*> compressed_childs_;
    std::vector<ArchivedLeaf> archived_childs_;
    std::vector<CompressedLeafNodeSession*> archived_sessions_tree_;
    std::vector<int> archived_sessions_timeout_; // ms
    std::vector<bool> modified_;

    SoftPoint::Ttime earlier_key_;
    bool ask_for_new_tree_;

    std::atomic<bool> running_;
    std::thread session_cleaner_;

    void createNewCompressedChild(SoftPoint::Ttime key);
    bool useNewTree();
    int getKeyIndex(SoftPoint::Ttime key);

    void loadStoredData();
    void insert(SoftPoint::Ttime key, const ArchivedLeaf& leaf);
    void archiveFirst();

    void createSession(size_t index);

    void clean();
  };

} // namespace mementar

#endif // MEMENTAR_ARCHIVEDLEAFNODE_H
