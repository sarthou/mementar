#ifndef MEMENTAR_COMPRESSEDLEAFNODE_H
#define MEMENTAR_COMPRESSEDLEAFNODE_H

#include <atomic>
#include <shared_mutex>
#include <thread>
#include <vector>

#include "mementar/core/LtManagement/EpisodicTree/CompressedLeaf.h"
#include "mementar/core/LtManagement/EpisodicTree/Context.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  class ArchivedLeaf;

  class CompressedLeafNode
  {
    friend ArchivedLeaf;
    using LeafType = typename BplusLeaf<SoftPoint::Ttime, Fact*>::LeafType;

  public:
    CompressedLeafNode(const std::string& directory);
    ~CompressedLeafNode();

    CompressedLeafNode* split();

    void insert(Fact* data);
    void remove(Fact* data);
    LeafType* find(const SoftPoint::Ttime& key);
    LeafType* findNear(const SoftPoint::Ttime& key);
    LeafType* getFirst();
    LeafType* getLast();

    void display(SoftPoint::Ttime key);
    size_t size() { return keys_.size(); }

    std::string getDirectory() { return directory_; }
    SoftPoint::Ttime getKey()
    {
      if(contexts_.empty() == false)
        return contexts_[0].getKey();
      else
        return -1;
    }

    void newSession() { ask_for_new_tree_ = true; }

  private:
    CompressedLeafNode() = default;
    void init();

    std::string directory_;
    mutable std::shared_timed_mutex mut_;

    // keys_.size() == btree_childs_.size() + compressed_childs_.size()
    // keys_[i] correspond to the first key of child i
    std::vector<SoftPoint::Ttime> keys_;
    std::vector<Context> contexts_;
    std::vector<BplusTree<SoftPoint::Ttime, Fact*>*> btree_childs_;
    std::vector<CompressedLeaf> compressed_childs_;
    std::vector<BplusTree<SoftPoint::Ttime, Fact*>*> compressed_sessions_tree_;
    std::vector<long> compressed_sessions_timeout_; // ms
    std::vector<bool> modified_;

    size_t last_tree_nb_leafs_;
    SoftPoint::Ttime earlier_key_;
    bool ask_for_new_tree_;

    std::atomic<bool> running_;
    std::thread session_cleaner_;

    inline void createNewTreeChild(const SoftPoint::Ttime& key);
    inline bool useNewTree();
    inline int getKeyIndex(const SoftPoint::Ttime& key);

    bool loadStoredData();
    void insert(const SoftPoint::Ttime& key, const CompressedLeaf& leaf);

    void compressFirst();
    void createSession(size_t index);

    void clean();
  };

} // namespace mementar

#endif // MEMENTAR_COMPRESSEDLEAFNODE_H
