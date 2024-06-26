#ifndef MEMENTAR_ACTIONREADER_H
#define MEMENTAR_ACTIONREADER_H

#include <map>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>

#include "mementar/core/memGraphs/Graphs/FactGraph.h"

namespace mementar {

  struct Action_t
  {
    explicit Action_t(const SoftPoint& point) : start(point),
                                                level(0) {}

    std::string name;
    SoftPoint start;
    std::optional<SoftPoint> end;

    size_t level;
  };

  class ActionReader
  {
  public:
    ActionReader();

    void read(FactGraph* graph, CvFont* font);

    std::map<std::string, Action_t> actions_;
    size_t max_level_;
    size_t max_text_size_;

  private:
    std::vector<Action*> actions_to_manage_;
    std::vector<Action*> actions_managed_;

    Action_t getAction(Action* action);
    void closeAction(Action* action);

    size_t getMinLevel();
    void setLevels();
    void getTextSize(const std::string& txt, CvFont* font);
  };

} // namespace mementar

#endif // MEMENTAR_ACTIONREADER_H
