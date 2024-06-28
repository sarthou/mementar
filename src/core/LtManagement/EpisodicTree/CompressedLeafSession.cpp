#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafSession.h"

#include <ctime>
#include <sstream>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"
#include "mementar/core/LtManagement/archiving_compressing/compressing/LzUncompress.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"

namespace mementar {

  CompressedLeafSession::CompressedLeafSession(const SoftPoint::Ttime& key, size_t index) : key_(key),
                                                                                            index_(index)
  {}

  BplusTree<SoftPoint::Ttime, Fact*>* CompressedLeafSession::getTree(const Header& header, Archive& arch) const
  {
    std::string comp = arch.extractFile(index_, header);

    LzUncompress lz;
    std::vector<char> comp_data(comp.begin(), comp.end());
    std::string out = lz.uncompress(comp_data);
    BplusTree<SoftPoint::Ttime, Fact*>* tree = new BplusTree<SoftPoint::Ttime, Fact*>();

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

  std::vector<char> CompressedLeafSession::getRawData(const Header& header, Archive& arch) const
  {
    std::string comp = arch.extractFile(index_, header);
    return std::vector<char>(comp.begin(), comp.end());
  }

} // namespace mementar
