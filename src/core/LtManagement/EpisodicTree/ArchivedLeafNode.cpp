#include "mementar/core/LtManagement/EpisodicTree/ArchivedLeafNode.h"

#include <chrono>
#include <cstddef>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafNode.h"
#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafNodeSession.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/graphical/Display.h"

namespace mementar {
  ArchivedLeafNode::ArchivedLeafNode(const std::string& directory,
                                     size_t order) : directory_(directory),
                                                     order_(order),
                                                     earlier_key_(0),
                                                     ask_for_new_tree_(false),
                                                     running_(false)
  {
    loadStoredData();

    running_ = true;
    session_cleaner_ = std::move(std::thread(&ArchivedLeafNode::clean, this));
  }

  ArchivedLeafNode::~ArchivedLeafNode()
  {
    running_ = false;
    session_cleaner_.join();

    mut_.lock();
    for(auto* tree : compressed_childs_)
      delete tree;

    Display::info("Archive trees:");
    size_t nb_leafs = archived_sessions_tree_.size();
    size_t leafs_cpt = 0;
    Display::percent(0);

    for(auto* tree : archived_sessions_tree_)
    {
      delete tree;
      Display::percent((++leafs_cpt) * 100 / nb_leafs);
    }

    Display::debug("");
    mut_.unlock();
  }

  void ArchivedLeafNode::insert(Fact* data)
  {
    mut_.lock_shared();
    if(keys_.empty())
    {
      mut_.unlock_shared();
      createNewCompressedChild(data->getTime());
      mut_.lock_shared();
      compressed_childs_[0]->insert(data);
    }
    else
    {
      if((time_t)data->getTime() < keys_[0])
      {
        Display::error("try to insert fact in past that do not exist");
        return;
      }

      size_t index = getKeyIndex(data->getTime());

      if(index < archived_childs_.size())
      {
        if((index == archived_childs_.size() - 1) && (keys_.size() == archived_childs_.size()) &&
           ((time_t)data->getTime() > earlier_key_))
        {
          mut_.unlock_shared();
          createNewCompressedChild(data->getTime());
          mut_.lock_shared();
          compressed_childs_[0]->insert(data);
        }
        else
        {
          mut_.unlock_shared();
          createSession(index);
          mut_.lock_shared();
          archived_sessions_tree_[index]->insert(data);
          modified_[index] = true;
        }
      }
      else if(useNewTree())
      {
        mut_.unlock_shared();
        mut_.lock();
        compressed_childs_.insert(compressed_childs_.end() - 1, compressed_childs_[0]->split());
        keys_.push_back(compressed_childs_[compressed_childs_.size() - 1]->getKey());
        mut_.unlock();
        mut_.lock_shared();
        compressed_childs_[compressed_childs_.size() - 1]->insert(data);

        // verify if a chld need to be compressed
        if(compressed_childs_.size() > 1)
        {
          mut_.unlock_shared();
          archiveFirst();
          mut_.lock_shared();
        }
      }
      else
      {
        compressed_childs_[index - archived_childs_.size()]->insert(data);
      }
    }
    mut_.unlock_shared();

    if(earlier_key_ < (time_t)data->getTime())
      earlier_key_ = data->getTime();
  }

  void ArchivedLeafNode::remove(Fact* data)
  {
    mut_.lock_shared();
    int index = getKeyIndex(data->getTime());
    if(index >= 0)
    {
      if((size_t)index < archived_childs_.size())
      {
        mut_.unlock_shared();
        createSession(index);
        mut_.lock_shared();
        if(archived_sessions_tree_[index]->remove(data))
          modified_[index] = true;
      }
      else
        compressed_childs_[index - archived_childs_.size()]->remove(data);
    }
    mut_.unlock_shared();
  }

  ArchivedLeafNode::LeafType* ArchivedLeafNode::find(const time_t& key)
  {
    ArchivedLeafNode::LeafType* res = nullptr;

    mut_.lock_shared();
    int index = getKeyIndex(key);
    if(index >= 0)
    {
      if((size_t)index < archived_childs_.size())
      {
        mut_.unlock_shared();
        createSession(index);
        mut_.lock_shared();
        res = archived_sessions_tree_[index]->find(key);
      }
      else
        res = compressed_childs_[index - archived_childs_.size()]->find(key);
    }
    mut_.unlock_shared();
    return res;
  }

  ArchivedLeafNode::LeafType* ArchivedLeafNode::findNear(const time_t& key)
  {
    ArchivedLeafNode::LeafType* res = nullptr;

    mut_.lock_shared();
    int index = getKeyIndex(key);
    if(index >= 0)
    {
      if((size_t)index < archived_childs_.size())
      {
        mut_.unlock_shared();
        createSession(index);
        mut_.lock_shared();
        res = archived_sessions_tree_[index]->findNear(key);
      }
      else
        res = compressed_childs_[index - archived_childs_.size()]->findNear(key);
    }
    mut_.unlock_shared();

    return res;
  }

  ArchivedLeafNode::LeafType* ArchivedLeafNode::getFirst()
  {
    ArchivedLeafNode::LeafType* res = nullptr;

    mut_.lock_shared();
    if(archived_childs_.empty() == false)
    {
      mut_.unlock_shared();
      createSession(0);
      mut_.lock_shared();
      res = archived_sessions_tree_[0]->getFirst();
    }
    else if(compressed_childs_.empty() == false)
      res = compressed_childs_[0]->getFirst();
    mut_.unlock_shared();

    return res;
  }

  ArchivedLeafNode::LeafType* ArchivedLeafNode::getLast()
  {
    ArchivedLeafNode::LeafType* res = nullptr;

    mut_.lock_shared();
    if(compressed_childs_.empty() == false)
      res = compressed_childs_[compressed_childs_.size() - 1]->getLast();
    else if(archived_childs_.empty() == false)
    {
      mut_.unlock_shared();
      createSession(archived_childs_.size() - 1);
      mut_.lock_shared();
      res = archived_sessions_tree_[archived_childs_.size() - 1]->getLast();
    }
    mut_.unlock_shared();

    return res;
  }

  void ArchivedLeafNode::display(time_t key)
  {
    mut_.lock_shared();
    int index = getKeyIndex(key);

    if(index >= 0)
    {
      if((size_t)index < archived_childs_.size())
        std::cout << archived_childs_[index].getDirectory() << ".mar" << std::endl;
      else
        compressed_childs_[index - archived_childs_.size()]->display(key);
    }
    mut_.unlock_shared();
  }

  void ArchivedLeafNode::newSession()
  {
    if(compressed_childs_.empty() == false)
      compressed_childs_[compressed_childs_.size() - 1]->newSession();
  }

  void ArchivedLeafNode::createNewCompressedChild(const time_t& key)
  {
    mut_.lock();
    compressed_childs_.push_back(new CompressedLeafNode(directory_));
    keys_.push_back(key);
    mut_.unlock();
  }

  bool ArchivedLeafNode::useNewTree()
  {
    if(compressed_childs_.empty())
      return false;
    else if(compressed_childs_[0]->size() >= order_)
      return true;
    else
      return false;
  }

  int ArchivedLeafNode::getKeyIndex(const time_t& key)
  {
    int index = (int)keys_.size() - 1;
    for(size_t i = 0; i < keys_.size(); i++)
    {
      if(key < keys_[i])
      {
        index = i - 1;
        break;
      }
    }
    return index;
  }

  void ArchivedLeafNode::loadStoredData()
  {
    Display::info("Load archived files:");
    size_t nb_file =
      std::distance(std::filesystem::directory_iterator(directory_), std::filesystem::directory_iterator{});
    size_t cpt_file = 0;
    Display::percent(0);

    for(const auto& entry : std::filesystem::directory_iterator(directory_))
    {
      std::string complete_dir = entry.path().generic_string();
      std::string dir = complete_dir.substr(directory_.size());
      if(dir[0] == '/')
        dir.erase(dir.begin());
      size_t dot_pose = dir.find('.');
      if(dot_pose != std::string::npos)
      {
        std::string ext = dir.substr(dot_pose + 1);
        std::string str_key = dir.substr(0, dot_pose);
        if(ext == "mar")
        {
          time_t key = 0;
          std::istringstream iss(str_key);
          iss >> key;
          insert(key, ArchivedLeaf(key, complete_dir));

          Display::debug(complete_dir);
        }
      }

      cpt_file++;
      Display::percent(cpt_file * 100 / nb_file);
    }
    Display::percent(100);
    Display::debug("");

    CompressedLeafNode* comp = new CompressedLeafNode(directory_);
    if(comp->getKey() != time_t(-1))
    {
      compressed_childs_.push_back(comp);
      keys_.push_back(comp->getKey());
      archived_sessions_tree_.push_back(nullptr);
      archived_sessions_timeout_.push_back(0);
      modified_.push_back(false);
    }
    else
      Display::warning("No compressed data loaded");

    if(archived_childs_.empty() == false)
    {
      createSession(archived_childs_.size() - 1);
      earlier_key_ = archived_sessions_tree_[archived_childs_.size() - 1]->getLast()->getKey();
    }
  }

  void ArchivedLeafNode::insert(const time_t& key, const ArchivedLeaf& leaf)
  {
    mut_.lock();
    if((keys_.empty()) || (key > keys_[keys_.size() - 1]))
    {
      keys_.push_back(key);
      archived_childs_.push_back(leaf);
      archived_sessions_tree_.push_back(nullptr);
      archived_sessions_timeout_.push_back(0);
      modified_.push_back(false);
    }
    else
    {
      for(int i = 0; i < (int)keys_.size(); i++)
      {
        if(key < keys_[i])
        {
          keys_.insert(keys_.begin() + i, key);
          archived_childs_.insert(archived_childs_.begin() + i, leaf);
          archived_sessions_tree_.insert(archived_sessions_tree_.begin() + i, nullptr);
          archived_sessions_timeout_.insert(archived_sessions_timeout_.begin() + i, 0);
          modified_.insert(modified_.begin() + i, false);
          break;
        }
      }
    }
    mut_.unlock();
  }

  void ArchivedLeafNode::archiveFirst()
  {
    if(compressed_childs_.empty())
      return;

    ArchivedLeaf tmp(compressed_childs_[0], order_ / 2, directory_);

    mut_.lock();
    archived_childs_.push_back(tmp);
    archived_sessions_tree_.push_back(nullptr);
    archived_sessions_timeout_.push_back(0);
    modified_.push_back(false);

    compressed_childs_.erase(compressed_childs_.begin());
    mut_.unlock();
  }

  void ArchivedLeafNode::createSession(size_t index)
  {
    mut_.lock();
    if(archived_sessions_tree_[index] == nullptr)
      archived_sessions_tree_[index] = new CompressedLeafNodeSession(archived_childs_[index].getDirectory());

    archived_sessions_timeout_[index] = std::time(nullptr);
    mut_.unlock();
  }

  void ArchivedLeafNode::clean()
  {
    size_t rate = 100000 / 1000; // clean rate / look up rate;
    size_t cpt = rate;
    while(running_ == true)
    {
      if(cpt-- == 0)
      {
        cpt = rate;
        time_t now = std::time(0);
        for(size_t i = 0; i < archived_sessions_timeout_.size(); i++)
        {
          if((archived_sessions_tree_[i] != nullptr) &&
             (std::difftime(now, archived_sessions_timeout_[i]) > 30)) // session expire after 30s
          {
            mut_.lock();
            delete archived_sessions_tree_[i];
            archived_sessions_tree_[i] = nullptr;
            mut_.unlock();
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(rate));
    }
  }

} // namespace mementar
