#ifndef MEMENTAR_COMPRESSEDLEAFNODESESSION_H
#define MEMENTAR_COMPRESSEDLEAFNODESESSION_H

#include <atomic>
#include <ctime>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafSession.h"
#include "mementar/core/LtManagement/EpisodicTree/Context.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  class CompressedLeafNodeSession
  {
    using LeafType = typename BplusLeaf<time_t, Fact*>::LeafType;

  public:
    CompressedLeafNodeSession(const std::string& file_name);
    ~CompressedLeafNodeSession();

    void insert(Fact* data);
    bool remove(Fact* data);
    LeafType* find(const time_t& key);
    LeafType* findNear(const time_t& key);
    LeafType* getFirst();
    LeafType* getLast();

    time_t getKey()
    {
      if(contexts_.empty() == false)
        return contexts_[0].getKey();
      else
        return -1;
    }

  private:
    std::string file_name_;
    mutable std::shared_timed_mutex mut_;

    Archive arch_;
    Header header_;

    // keys_.size() == btree_childs_.size() + compressed_childs_.size()
    // keys_[i] correspond to the first key of child i
    std::vector<Context> contexts_;
    std::vector<CompressedLeafSession> childs_;
    std::vector<BplusTree<time_t, Fact*>*> sessions_tree_;
    std::vector<bool> modified_;

    time_t earlier_key_;

    inline int getKeyIndex(const time_t& key);

    void loadData();

    void createSession(size_t index);

    std::vector<char> treeToRaw(size_t index);
  };

} // namespace mementar

#endif // MEMENTAR_COMPRESSEDLEAFNODESESSION_H
