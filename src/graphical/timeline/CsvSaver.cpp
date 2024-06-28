#include "mementar/graphical/timeline/CsvSaver.h"

#include <cstddef>
#include <fstream>
#include <iostream>
#include <string>

#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"
#include "mementar/core/memGraphs/Timeline.h"

namespace mementar {

  bool CsvSaver::save(const std::string& file_name, Timeline* timeline)
  {
    std::cout << "Save Timeline to csv file: " << file_name << std::endl;
    auto* tree = timeline->facts.getTimeline();
    auto* node = static_cast<BplusLeaf<SoftPoint::Ttime, ContextualizedFact*>*>(tree->getFirst());

    if(timeline->facts.getTimeline()->getFirst() == nullptr)
      return false;

    std::ofstream file_stream;
    std::string separator = ";";
    std::string triplet_separator = "|";

    file_stream.open(file_name);
    if(file_stream.is_open() == false)
    {
      std::cout << "Fail to open file: " << file_name << std::endl;
      return false;
    }
    file_stream << "time;type;subject;property;object\n";
    while(node != nullptr)
    {
      double time = node->getData()[0]->getTime();
      for(auto* fact : node->getData())
      {
        file_stream << std::to_string(time) << separator;
        std::string triplet_str = fact->Triplet::toString();
        replaceAll(triplet_str, ";", ",");
        replaceAll(triplet_str, triplet_separator, separator);
        replaceAll(triplet_str, "]", "]" + separator);
        file_stream << triplet_str << "\n";
      }
      node = node->getNextLeaf();
    }

    file_stream.close();
    return true;
  }

  void CsvSaver::replaceAll(std::string& text, const std::string& to_replace, const std::string& replace_with)
  {
    size_t pose = text.find(to_replace);
    while(pose != std::string::npos)
    {
      text.replace(pose, to_replace.size(), replace_with);
      pose = text.find(to_replace, pose + replace_with.size());
    }
  }

} // namespace mementar