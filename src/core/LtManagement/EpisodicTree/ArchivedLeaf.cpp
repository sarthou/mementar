#include "mementar/core/LtManagement/EpisodicTree/ArchivedLeaf.h"

#include <cstddef>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafNode.h"
#include "mementar/core/LtManagement/EpisodicTree/Context.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"
#include "mementar/core/LtManagement/archiving_compressing/compressing/LzUncompress.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"

namespace mementar {

  ArchivedLeaf::ArchivedLeaf(CompressedLeafNode* tree, size_t nb, const std::string& directory)
  {
    if(tree == nullptr)
      return;

    key_ = tree->getFirst()->getKey();
    directory_ = directory + '/' + std::to_string(key_);

    std::vector<SoftPoint::Ttime> keys;
    std::vector<Context> contexts;
    std::vector<std::string> input_files;

    for(size_t i = 0; i < nb; i++)
    {
      if(i >= tree->compressed_childs_.size())
        break;

      keys.push_back(tree->keys_[i]);
      contexts.push_back(tree->contexts_[i]);
      input_files.push_back(tree->compressed_childs_[i].getDirectory() + ".mlz");
    }

    std::string context = Context::contextsToString(contexts);

    Archive arch(context, input_files);

    std::vector<char> data = arch.load();

    arch.saveToFile(data, directory_);

    for(size_t i = 0; i < nb; i++)
    {
      if(i >= tree->compressed_childs_.size())
        break;

      std::filesystem::remove(tree->compressed_childs_[i].getDirectory() + ".mlz");
    }
  }

  ArchivedLeaf::ArchivedLeaf(const SoftPoint::Ttime& key, const std::string& directory) : key_(key)
  {
    size_t dot_pose = directory.find('.');
    directory_ = directory.substr(0, dot_pose);
  }

  BplusTree<SoftPoint::Ttime, Fact*>* ArchivedLeaf::getTree(size_t i)
  {
    mementar::Archive arch;
    std::cout << "ArchivedLeaf::getTree READ BINARY FILE " << directory_ << ".mar" << std::endl;
    arch.readBinaryFile(directory_ + ".mar");
    mementar::Header header = arch.getHeader();

    std::string comp = arch.extractFile(i, header);

    LzUncompress lz;
    std::vector<char> comp_data(comp.begin(), comp.end());
    std::string out = lz.uncompress(comp_data);
    BplusTree<SoftPoint::Ttime, Fact*>* tree = new BplusTree<SoftPoint::Ttime, Fact*>();

    std::istringstream iss(out);
    std::string line;
    while(std::getline(iss, line))
    {
      Fact* fact = Fact::deserializePtr(line);
      if(fact != nullptr)
        tree->insert(fact->getTime(), fact);
    }

    return tree;
  }

  std::vector<Context> ArchivedLeaf::getContexts()
  {
    mementar::Archive arch;
    std::cout << "ArchivedLeaf::getContexts READ BINARY FILE " << directory_ << ".mar" << std::endl;
    arch.readBinaryFile(directory_ + ".mar");
    mementar::Header header = arch.getHeader();

    std::string out = arch.extractDescription(header);

    return Context::stringToContext(out);
  }

} // namespace mementar
