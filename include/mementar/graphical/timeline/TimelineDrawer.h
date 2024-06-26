#ifndef MEMENTAR_TIMELINEDRAWER_H
#define MEMENTAR_TIMELINEDRAWER_H

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgcodecs.hpp>
#include <set>
#include <string>

#include "mementar/core/memGraphs/Timeline.h"
#include "mementar/graphical/timeline/ActionReader.h"
#include "mementar/graphical/timeline/FactReader.h"

namespace mementar {

  class TimelineDrawer
  {
  public:
    TimelineDrawer() : image_(nullptr) {}

    bool draw(const std::string& file_name, Timeline* timeline);

  private:
    IplImage* image_;

    std::set<size_t> used_poses_;

    void drawVector(int start, int end, int pose, CvFont* font);
    void drawAction(const Action_t& action, int line_pose, int max_level, int start_time, int end_time, CvFont* font);
    void drawEvent(const Fact_t& event, int line_pose, int start_time, CvFont* font);

    int getTextSize(const std::string& txt, CvFont* font);
    void drawElipseStart(int x, int y, CvScalar color);
    void drawElipseEnd(int x, int y, CvScalar color);

    CvScalar scalarHSV2BGR(float H, float S, float V);
  };

} // namespace mementar

#endif // MEMENTAR_TIMELINEDRAWER_H
