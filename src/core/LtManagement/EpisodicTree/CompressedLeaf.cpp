#include "mementar/core/LtManagement/EpisodicTree/CompressedLeaf.h"

#include <ctime>
#include <sstream>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/compressing/LzCompress.h"
#include "mementar/core/LtManagement/archiving_compressing/compressing/LzUncompress.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  CompressedLeaf::CompressedLeaf(BplusTree<time_t, Fact*>* tree, const std::string& directory)
  {
    if(tree == nullptr)
      return;

    key_ = tree->getFirst()->getKey();
    directory_ = directory + '/' + std::to_string(key_);

    LzCompress lz_comp;
    std::string in = treeToString(tree);
    std::vector<char> out_vect = lz_comp.compress(in);

    lz_comp.displayCompressionRate(in.size(), out_vect.size());
    lz_comp.saveToFile(out_vect, directory_);
  }

  CompressedLeaf::CompressedLeaf(const time_t& key, const std::string& directory) : key_(key)
  {
    size_t dot_pose = directory.find('.');
    directory_ = directory.substr(0, dot_pose);
  }

  BplusTree<time_t, Fact*>* CompressedLeaf::getTree()
  {
    LzUncompress lz;
    std::vector<char> data;
    if(lz.readBinaryFile(data, directory_ + ".mlz"))
    {
      std::string out = lz.uncompress(data);
      BplusTree<time_t, Fact*>* tree = new BplusTree<time_t, Fact*>();

      std::istringstream iss(out);
      std::string line;
      while(std::getline(iss, line))
      {
        Fact* event = Fact::deserializePtr(line);
        if(event != nullptr)
          tree->insert(event->getTime(), event);
      }

      return tree;
    }

    return nullptr;
  }

  std::string CompressedLeaf::treeToString(BplusTree<time_t, Fact*>* tree)
  {
    std::string res;
    std::vector<Fact*> tmp_data;
    BplusLeaf<time_t, Fact*>* it = tree->getFirst();
    while(it != nullptr)
    {
      tmp_data = it->getData();
      for(auto& data : tmp_data)
        res += Fact::serialize(data) + "\n";
      it = static_cast<BplusLeaf<time_t, Fact*>*>(it->getNextLeaf());
    }

    return res;
  }

} // namespace mementar
