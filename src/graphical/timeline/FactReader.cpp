#include "mementar/graphical/timeline/FactReader.h"

#include <cstddef>
#include <string>

#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"
#include "mementar/core/memGraphs/Graphs/FactGraph.h"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc_c.h"

namespace mementar {

  void FactReader::read(FactGraph* graph, CvFont* font)
  {
    max_text_size_ = 0;

    auto* tree = graph->getTimeline();
    auto* node = static_cast<BplusLeaf<SoftPoint::Ttime, ContextualizedFact*>*>(tree->getFirst());

    while(node != nullptr)
    {
      Fact_t group_fact(node->getData()[0]->getTime());
      for(auto* fact : node->getData())
      {
        if(fact->isPartOfAction() == false)
        {
          // We do not display teh fact coming from forward deduction
          if(fact->getDeductionLevel() != 1)
          {
            group_fact.data += (group_fact.data.empty() ? "" : " -- ") + fact->Triplet::toString();
            if(fact->getTransitionDuration() > group_fact.time_point.getTransitionDuration())
              group_fact.time_point = SoftPoint(fact);
          }
        }
      }

      if(group_fact.data.empty() == false)
      {
        facts.push_back(group_fact);
        getTextSize(group_fact.data, font);
      }

      node = node->getNextLeaf();
    }
  }

  void FactReader::getTextSize(const std::string& txt, CvFont* font)
  {
    CvSize size;
    int baseline = 0;
    cvGetTextSize(txt.c_str(), font, &size, &baseline);
    size_t text_width = size.width;
    if(text_width > max_text_size_)
      max_text_size_ = text_width;
  }

} // namespace mementar
