#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafNodeSession.h"

#include <cstddef>
#include <ctime>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/EpisodicTree/Context.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"
#include "mementar/core/LtManagement/archiving_compressing/compressing/LzCompress.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"
#include "mementar/graphical/Display.h"

namespace mementar {

  CompressedLeafNodeSession::CompressedLeafNodeSession(const std::string& file_name) : file_name_(file_name),
                                                                                       earlier_key_(0)
  {
    loadData();
  }

  CompressedLeafNodeSession::~CompressedLeafNodeSession()
  {
    mut_.lock();
    std::string description = Context::contextsToString(contexts_);

    Archive arch(description, header_);

    std::vector<std::string> raw_datas;
    for(size_t i = 0; i < childs_.size(); i++)
    {
      if(sessions_tree_[i] != nullptr)
      {
        if(modified_[i])
        {
          auto tmp_raw = treeToRaw(i);
          raw_datas.emplace_back(tmp_raw.begin(), tmp_raw.end());
        }
        else
        {
          auto tmp_raw = childs_[i].getRawData(header_, arch_);
          raw_datas.emplace_back(tmp_raw.begin(), tmp_raw.end());
        }
      }
      else
      {
        auto tmp_raw = childs_[i].getRawData(header_, arch_);
        raw_datas.emplace_back(tmp_raw.begin(), tmp_raw.end());
      }
    }

    std::vector<char> data = arch.load(raw_datas);

    arch.saveToFile(data, file_name_);

    mut_.unlock();
  }

  int CompressedLeafNodeSession::getKeyIndex(const time_t& key)
  {
    int index = (int)contexts_.size() - 1;
    for(size_t i = 0; i < contexts_.size(); i++)
    {
      if(key < contexts_[i].getKey())
      {
        index = (int)i - 1;
        break;
      }
    }
    return index;
  }

  void CompressedLeafNodeSession::insert(Fact* data)
  {
    mut_.lock_shared();
    if(contexts_.empty())
    {
      Display::error("Can not insert in empty session");
    }
    else
    {
      if((time_t)data->getTime() < contexts_[0].getKey())
      {
        Display::error("try to insert fact in past that do not exist");
        return;
      }

      size_t index = getKeyIndex(data->getTime());
      mut_.unlock_shared();
      createSession(index);
      mut_.lock_shared();
      sessions_tree_[index]->insert(data->getTime(), data);
      contexts_[index].insert(data);
      modified_[index] = true;
    }

    mut_.unlock_shared();
  }

  bool CompressedLeafNodeSession::remove(Fact* data)
  {
    bool res = false;
    mut_.lock_shared();
    int index = getKeyIndex(data->getTime());
    if(index >= 0)
    {
      mut_.unlock_shared();
      createSession(index);
      mut_.lock_shared();
      if(sessions_tree_[index]->remove(data->getTime(), data))
      {
        modified_[index] = true;
        contexts_[index].remove(data);
        res = true;
      }
    }
    mut_.unlock_shared();
    return res;
  }

  CompressedLeafNodeSession::LeafType* CompressedLeafNodeSession::find(const time_t& key)
  {
    CompressedLeafNodeSession::LeafType* res = nullptr;

    mut_.lock_shared();
    int index = getKeyIndex(key);
    if(index >= 0)
    {
      mut_.unlock_shared();
      createSession(index);
      mut_.lock_shared();
      res = sessions_tree_[index]->find(key);
    }
    mut_.unlock_shared();
    return res;
  }

  CompressedLeafNodeSession::LeafType* CompressedLeafNodeSession::findNear(const time_t& key)
  {
    CompressedLeafNodeSession::LeafType* res = nullptr;

    mut_.lock_shared();
    int index = getKeyIndex(key);
    if(index >= 0)
    {
      mut_.unlock_shared();
      createSession(index);
      mut_.lock_shared();
      res = sessions_tree_[index]->findNear(key);
    }
    mut_.unlock_shared();

    return res;
  }

  CompressedLeafNodeSession::LeafType* CompressedLeafNodeSession::getFirst()
  {
    CompressedLeafNodeSession::LeafType* res = nullptr;

    createSession(0);
    mut_.lock_shared();
    res = sessions_tree_[0]->getFirst();
    mut_.unlock_shared();

    return res;
  }

  CompressedLeafNodeSession::LeafType* CompressedLeafNodeSession::getLast()
  {
    CompressedLeafNodeSession::LeafType* res = nullptr;

    createSession(childs_.size() - 1);
    mut_.lock_shared();
    res = sessions_tree_[childs_.size() - 1]->getLast();
    mut_.unlock_shared();

    return res;
  }

  void CompressedLeafNodeSession::loadData()
  {
    arch_.readBinaryFile(file_name_ + ".mar");
    header_ = arch_.getHeader();

    std::string description = arch_.extractDescription(header_);
    contexts_ = Context::stringToContext(description);

    // assume as ordered ?
    for(size_t i = 0; i < contexts_.size(); i++)
    {
      childs_.emplace_back(contexts_[i].getKey(), i);
      sessions_tree_.push_back(nullptr);
      modified_.push_back(false);
    }

    if(childs_.empty() == false)
    {
      createSession(childs_.size() - 1);
      earlier_key_ = sessions_tree_[childs_.size() - 1]->getLast()->getKey();
    }
  }

  void CompressedLeafNodeSession::createSession(size_t index)
  {
    mut_.lock();
    if(sessions_tree_[index] == nullptr)
      sessions_tree_[index] = childs_[index].getTree(header_, arch_);
    mut_.unlock();
  }

  std::vector<char> CompressedLeafNodeSession::treeToRaw(size_t index)
  {
    std::string res;

    std::vector<Fact*> tmp_data;
    BplusLeaf<time_t, Fact*>* it = sessions_tree_[index]->getFirst();
    while(it != nullptr)
    {
      tmp_data = it->getData();
      for(auto& data : tmp_data)
        res += Fact::serialize(data) + "\n";
      it = static_cast<BplusLeaf<time_t, Fact*>*>(it->getNextLeaf());
    }

    mementar::LzCompress lz_comp;
    return lz_comp.compress(res);
  }

} // namespace mementar
