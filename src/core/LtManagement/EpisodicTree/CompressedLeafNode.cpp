#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafNode.h"

#include <chrono>
#include <cstddef>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "mementar/core/LtManagement/EpisodicTree/CompressedLeaf.h"
#include "mementar/core/LtManagement/EpisodicTree/Context.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"
#include "mementar/graphical/Display.h"

namespace mementar {

  CompressedLeafNode::CompressedLeafNode(const std::string& directory) : directory_(directory),
                                                                         last_tree_nb_leafs_(0),
                                                                         earlier_key_(0),
                                                                         ask_for_new_tree_(false)
  {
    if(loadStoredData())
      Context::loadContexts(contexts_, directory_);

    running_ = true;
    session_cleaner_ = std::move(std::thread(&CompressedLeafNode::clean, this));
  }

  CompressedLeafNode::~CompressedLeafNode()
  {
    running_ = false;
    session_cleaner_.join();

    Context::storeContexts(contexts_, directory_);

    mut_.lock();
    Display::info("Compress trees:");
    size_t nb_leafs = keys_.size();
    size_t leafs_cpt = 0;
    Display::percent(0);

    for(auto* tree : btree_childs_)
    {
      compressed_childs_.emplace_back(tree, directory_);
      delete tree;
      Display::percent((++leafs_cpt) * 100 / nb_leafs);
    }

    for(size_t i = 0; i < compressed_sessions_tree_.size(); i++)
    {
      if(compressed_sessions_tree_[i] != nullptr)
      {
        if(modified_[i])
          compressed_childs_[i] = std::move(CompressedLeaf(compressed_sessions_tree_[i], directory_));
        delete compressed_sessions_tree_[i];
      }
      Display::percent((++leafs_cpt) * 100 / nb_leafs);
    }
    Display::debug("");
    mut_.unlock();
  }

  CompressedLeafNode* CompressedLeafNode::split()
  {
    mut_.lock();
    CompressedLeafNode* new_one = new CompressedLeafNode();
    new_one->directory_ = directory_;

    size_t nb = keys_.size() / 2;
    for(size_t i = 0; i < nb; i++)
    {
      new_one->keys_.push_back(keys_[0]);
      new_one->contexts_.push_back(contexts_[0]);
      new_one->compressed_childs_.push_back(compressed_childs_[0]);
      new_one->compressed_sessions_tree_.push_back(compressed_sessions_tree_[0]);
      new_one->compressed_sessions_timeout_.push_back(compressed_sessions_timeout_[0]);
      new_one->modified_.push_back(modified_[0]);

      keys_.erase(keys_.begin());
      contexts_.erase(contexts_.begin());
      compressed_childs_.erase(compressed_childs_.begin());
      compressed_sessions_tree_.erase(compressed_sessions_tree_.begin());
      compressed_sessions_timeout_.erase(compressed_sessions_timeout_.begin());
      modified_.erase(modified_.begin());
    }

    new_one->init();
    mut_.unlock();

    return new_one;
  }

  void CompressedLeafNode::insert(Fact* data)
  {
    mut_.lock_shared();
    if(keys_.empty())
    {
      mut_.unlock_shared();
      createNewTreeChild(data->getTime());
      mut_.lock_shared();
      last_tree_nb_leafs_ = btree_childs_[0]->insert(data->getTime(), data);
      contexts_[0].insert(data);
    }
    else
    {
      if((SoftPoint::Ttime)data->getTime() < keys_[0])
      {
        Display::error("try to insert fact in past that do not exist");
        return;
      }

      size_t index = getKeyIndex(data->getTime());

      if(index < compressed_childs_.size())
      {
        if((index == compressed_childs_.size() - 1) && (keys_.size() == compressed_childs_.size()) && ((SoftPoint::Ttime)data->getTime() > earlier_key_))
        {
          mut_.unlock_shared();
          createNewTreeChild(data->getTime());
          mut_.lock_shared();
          last_tree_nb_leafs_ = btree_childs_[0]->insert(data->getTime(), data);
          contexts_[keys_.size() - 1].insert(data);
        }
        else
        {
          mut_.unlock_shared();
          createSession(index);
          mut_.lock_shared();
          compressed_sessions_tree_[index]->insert(data->getTime(), data);
          contexts_[index].insert(data);
          modified_[index] = true;
        }
      }
      else if(useNewTree())
      {
        mut_.unlock_shared();
        createNewTreeChild(data->getTime());
        mut_.lock_shared();
        last_tree_nb_leafs_ = btree_childs_[btree_childs_.size() - 1]->insert(data->getTime(), data);
        contexts_[keys_.size() - 1].insert(data);

        // verify if a chld need to be compressed
        if(btree_childs_.size() > 2)
        {
          mut_.unlock_shared();
          compressFirst();
          mut_.lock_shared();
        }
      }
      else if(index - keys_.size() + 1 == 0) // if insert in more recent tree
      {
        last_tree_nb_leafs_ = btree_childs_[index - compressed_childs_.size()]->insert(data->getTime(), data);
        contexts_[index].insert(data);
      }
      else
      {
        btree_childs_[index - compressed_childs_.size()]->insert(data->getTime(), data);
        contexts_[index].insert(data);
      }
    }
    mut_.unlock_shared();

    if(earlier_key_ < (SoftPoint::Ttime)data->getTime())
      earlier_key_ = data->getTime();
  }

  void CompressedLeafNode::remove(Fact* data)
  {
    mut_.lock_shared();
    int index = getKeyIndex(data->getTime());
    if(index >= 0)
    {
      if((size_t)index < compressed_childs_.size())
      {
        mut_.unlock_shared();
        createSession(index);
        mut_.lock_shared();
        if(compressed_sessions_tree_[index]->remove(data->getTime(), data))
        {
          modified_[index] = true;
          contexts_[index].remove(data);
        }
      }
      else
      {
        if(btree_childs_[index - compressed_childs_.size()]->remove(data->getTime(), data))
          contexts_[index].remove(data);
      }
    }
    mut_.unlock_shared();
  }

  CompressedLeafNode::LeafType* CompressedLeafNode::find(const SoftPoint::Ttime& key)
  {
    CompressedLeafNode::LeafType* res = nullptr;

    mut_.lock_shared();
    int index = getKeyIndex(key);
    if(index >= 0)
    {
      if((size_t)index < compressed_childs_.size())
      {
        mut_.unlock_shared();
        createSession(index);
        mut_.lock_shared();
        res = compressed_sessions_tree_[index]->find(key);
      }
      else
        res = btree_childs_[index - compressed_childs_.size()]->find(key);
    }
    mut_.unlock_shared();
    return res;
  }

  CompressedLeafNode::LeafType* CompressedLeafNode::findNear(const SoftPoint::Ttime& key)
  {
    CompressedLeafNode::LeafType* res = nullptr;

    mut_.lock_shared();
    int index = getKeyIndex(key);
    if(index >= 0)
    {
      if((size_t)index < compressed_childs_.size())
      {
        mut_.unlock_shared();
        createSession(index);
        mut_.lock_shared();
        res = compressed_sessions_tree_[index]->findNear(key);
      }
      else
        res = btree_childs_[index - compressed_childs_.size()]->findNear(key);
    }
    mut_.unlock_shared();

    return res;
  }

  CompressedLeafNode::LeafType* CompressedLeafNode::getFirst()
  {
    CompressedLeafNode::LeafType* res = nullptr;

    mut_.lock_shared();
    if(compressed_childs_.empty() == false)
    {
      mut_.unlock_shared();
      createSession(0);
      mut_.lock_shared();
      res = compressed_sessions_tree_[0]->getFirst();
    }
    else if(btree_childs_.empty() == false)
      res = btree_childs_[0]->getFirst();
    mut_.unlock_shared();

    return res;
  }

  CompressedLeafNode::LeafType* CompressedLeafNode::getLast()
  {
    CompressedLeafNode::LeafType* res = nullptr;

    mut_.lock_shared();
    if(btree_childs_.empty() == false)
      res = btree_childs_[btree_childs_.size() - 1]->getLast();
    else if(compressed_childs_.empty() == false)
    {
      mut_.unlock_shared();
      createSession(compressed_childs_.size() - 1);
      mut_.lock_shared();
      res = compressed_sessions_tree_[compressed_childs_.size() - 1]->getLast();
    }
    mut_.unlock_shared();

    return res;
  }

  void CompressedLeafNode::display(SoftPoint::Ttime key)
  {
    mut_.lock_shared();
    int index = getKeyIndex(key);

    if(index >= 0)
    {
      if((size_t)index < compressed_childs_.size())
        std::cout << compressed_childs_[index].getDirectory() << std::endl;
      else
        btree_childs_[index - compressed_childs_.size()]->displayTree();
    }
    mut_.unlock_shared();
  }

  void CompressedLeafNode::init()
  {
    running_ = true;
    session_cleaner_ = std::move(std::thread(&CompressedLeafNode::clean, this));
  }

  void CompressedLeafNode::createNewTreeChild(const SoftPoint::Ttime& key)
  {
    mut_.lock();
    btree_childs_.push_back(new BplusTree<SoftPoint::Ttime, Fact*>());
    keys_.push_back(key);
    contexts_.emplace_back(key);
    mut_.unlock();
  }

  bool CompressedLeafNode::useNewTree()
  {
    if(ask_for_new_tree_)
    {
      ask_for_new_tree_ = false;
      return true;
    }
    else if(last_tree_nb_leafs_ >= 100000)
      return true;
    else
      return false;
  }

  int CompressedLeafNode::getKeyIndex(const SoftPoint::Ttime& key)
  {
    int index = (int)keys_.size() - 1;
    for(size_t i = 0; i < keys_.size(); i++)
    {
      if(key < keys_[i])
      {
        index = (int)i - 1;
        break;
      }
    }
    return index;
  }

  bool CompressedLeafNode::loadStoredData()
  {
    size_t nb_file = std::distance(std::filesystem::directory_iterator(directory_), std::filesystem::directory_iterator{});
    if(nb_file != 0)
      Display::info("Load compressed files:");
    else
      return false;

    size_t cpt_file = 0;
    Display::percent(0);

    for(const auto& entry : std::filesystem::directory_iterator(directory_))
    {
      // std::filesystem::path to std::string conversion is platform-dependent:
      //   it needs std::string on POSIX and std::wstring on Windows. Futhermore,
      //   the directory separator change between operating systems.
      // To solve this, we force the conversion to std::string ('/' is used as
      //   directory separator).
      std::string complete_dir = entry.path().generic_string();
      std::string dir = complete_dir.substr(directory_.size());
      if(dir[0] == '/')
        dir.erase(dir.begin());
      size_t dot_pose = dir.find('.');
      if(dot_pose != std::string::npos)
      {
        std::string ext = dir.substr(dot_pose + 1);
        std::string str_key = dir.substr(0, dot_pose);
        if(ext == "mlz")
        {
          SoftPoint::Ttime key = 0;
          std::istringstream iss(str_key);
          iss >> key;
          insert(key, CompressedLeaf(key, complete_dir));

          Display::debug(complete_dir);
        }
      }

      cpt_file++;
      Display::percent(cpt_file * 100 / nb_file);
    }
    Display::percent(100);
    Display::debug("");

    if(compressed_childs_.empty() == false)
    {
      createSession(compressed_childs_.size() - 1);
      earlier_key_ = compressed_sessions_tree_[compressed_childs_.size() - 1]->getLast()->getKey();
    }

    if(compressed_childs_.empty() == false)
      return true;
    else
      return false;
  }

  void CompressedLeafNode::insert(const SoftPoint::Ttime& key, const CompressedLeaf& leaf)
  {
    mut_.lock();
    if(keys_.empty() || (key > keys_[keys_.size() - 1]))
    {
      keys_.push_back(key);
      contexts_.emplace_back(key);
      compressed_childs_.push_back(leaf);
      compressed_sessions_tree_.push_back(nullptr);
      compressed_sessions_timeout_.push_back(0);
      modified_.push_back(false);
    }
    else
    {
      for(int i = 0; i < (int)keys_.size(); i++)
      {
        if(key < keys_[i])
        {
          keys_.insert(keys_.begin() + i, key);
          compressed_childs_.insert(compressed_childs_.begin() + i, leaf);
          contexts_.insert(contexts_.begin() + i, Context(key));
          compressed_sessions_tree_.insert(compressed_sessions_tree_.begin() + i, nullptr);
          compressed_sessions_timeout_.insert(compressed_sessions_timeout_.begin() + i, 0);
          modified_.insert(modified_.begin() + i, false);
          break;
        }
      }
    }
    mut_.unlock();
  }

  void CompressedLeafNode::compressFirst()
  {
    if(btree_childs_.empty())
      return;

    CompressedLeaf tmp(btree_childs_[0], directory_);

    mut_.lock();
    compressed_childs_.push_back(tmp);
    compressed_sessions_tree_.push_back(nullptr);
    compressed_sessions_timeout_.push_back(0);
    modified_.push_back(false);

    btree_childs_.erase(btree_childs_.begin());
    mut_.unlock();
  }

  void CompressedLeafNode::createSession(size_t index)
  {
    mut_.lock();
    if(compressed_sessions_tree_[index] == nullptr)
      compressed_sessions_tree_[index] = compressed_childs_[index].getTree();

    compressed_sessions_timeout_[index] = std::time(nullptr);
    mut_.unlock();
  }

  void CompressedLeafNode::clean()
  {
    size_t rate = 100000 / 1000; // clean rate / look up rate;
    size_t cpt = rate;
    while(running_ == true)
    {
      if(cpt-- == 0)
      {
        cpt = rate;
        std::time_t now = std::time(nullptr);
        for(size_t i = 0; i < compressed_sessions_timeout_.size(); i++)
        {
          if((compressed_sessions_tree_[i] != nullptr) &&
             (std::difftime(now, compressed_sessions_timeout_[i]) > 30)) // session expire after 30s
          {
            mut_.lock();
            if(modified_[i])
              compressed_childs_[i] = std::move(CompressedLeaf(compressed_sessions_tree_[i], directory_));
            delete compressed_sessions_tree_[i];
            compressed_sessions_tree_[i] = nullptr;
            mut_.unlock();
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(rate));
    }
  }

} // namespace mementar
