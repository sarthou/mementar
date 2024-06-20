#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafSession.h"

#include <ctime>
#include <sstream>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"
#include "mementar/core/LtManagement/archiving_compressing/compressing/LzUncompress.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  CompressedLeafSession::CompressedLeafSession(const time_t& key, size_t index) : key_(key),
                                                                                  index_(index)
  {}

  BplusTree<time_t, Fact*>* CompressedLeafSession::getTree(Header& header, Archive& arch) const
  {
    std::string comp = arch.extractFile(index_, header);

    LzUncompress lz;
    std::vector<char> comp_data(comp.begin(), comp.end());
    std::string out = lz.uncompress(comp_data);
    BplusTree<time_t, Fact*>* tree = new BplusTree<time_t, Fact*>();

    std::istringstream iss(out);
    std::string line;
    while(std::getline(iss, line))
    {
      Fact* event = Fact::deserializePtr(line);
      if(event != nullptr)
        tree->insert((long)event->getTime(), event);
    }

    return tree;
  }

  std::vector<char> CompressedLeafSession::getRawData(Header& header, Archive& arch) const
  {
    std::string comp = arch.extractFile(index_, header);
    return std::vector<char>(comp.begin(), comp.end());
  }

} // namespace mementar
