#include "mementar/graphical/timeline/TimelineDrawer.h"

#include <cstddef>
#include <iostream>
#include <opencv2/core/core_c.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <optional>
#include <string>

#include "mementar/core/memGraphs/Timeline.h"
#include "mementar/graphical/timeline/ActionReader.h"
#include "mementar/graphical/timeline/FactReader.h"

#define UNIT_SPACE 35
#define SIDE_SPACE 120
#define EDGE_RADIUS 15
#define TEXT_WIDTH 350
#define MARK_WIDTH 10
#define IMAGE_MARGIN 25

namespace mementar {

  bool TimelineDrawer::draw(const std::string& file_name, Timeline* timeline)
  {
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 0, 1);

    used_poses_.clear();

    ActionReader actions_reader;
    actions_reader.read(&timeline->facts, &font);
    FactReader facts_reader;
    facts_reader.read(&timeline->facts, &font);

    if(timeline->facts.getTimeline()->getFirst() == nullptr)
      return false;

    int start = (int)timeline->facts.getTimeline()->getFirst()->getKey();
    int end = (int)timeline->facts.getTimeline()->getLast()->getKey() + 2;
    int width = (int)(actions_reader.max_level_ + 1) * SIDE_SPACE + (int)actions_reader.max_text_size_ + IMAGE_MARGIN * 3 + SIDE_SPACE + (int)facts_reader.max_text_size_;
    int height = (end - start) * UNIT_SPACE + IMAGE_MARGIN * 2;

    std::cout << "image size = " << width << " : " << height << std::endl;
    image_ = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    cvSet(image_, cvScalar(255, 255, 255));

    int line_pose = (int)(actions_reader.max_level_ + 1) * SIDE_SPACE + (int)actions_reader.max_text_size_ + IMAGE_MARGIN * 2;

    drawVector(start, end, line_pose, &font);

    std::cout << "---ACTIONS----" << std::endl;
    for(const auto& act : actions_reader.actions_)
      drawAction(act.second, line_pose, (int)actions_reader.max_level_, start, end, &font);

    std::cout << "---FACTS----" << std::endl;
    for(const auto& evt : facts_reader.facts)
      drawEvent(evt, line_pose, start, &font);

    if(file_name.empty() == false)
    {
      std::cout << "Save image " << file_name << std::endl;

      if((height != 1) && (width != 1))
        cv::imwrite(file_name, cv::cvarrToMat(image_));

      if(image_ != nullptr)
        cvReleaseImage(&image_);

      return true;
    }
    else
      return false;
  }

  void TimelineDrawer::drawVector(int start, int end, int pose, CvFont* font)
  {
    cvLine(image_, cvPoint(pose, IMAGE_MARGIN),
           cvPoint(pose, IMAGE_MARGIN + (end - start) * UNIT_SPACE),
           cvScalar(50, 50, 50), 2);

    for(size_t i = 0; i < (end - start) + 1; i++)
    {
      cvLine(image_, cvPoint(pose - MARK_WIDTH / 2, IMAGE_MARGIN + i * UNIT_SPACE),
             cvPoint(pose + MARK_WIDTH / 2, IMAGE_MARGIN + i * UNIT_SPACE),
             cvScalar(50, 50, 50), 2);

      std::string txt_num = std::to_string(start + i);

      cvPutText(image_, txt_num.c_str(), cvPoint(pose - getTextSize(txt_num, font) - 2, IMAGE_MARGIN + i * UNIT_SPACE - 2), font,
                cvScalar(50, 50, 50));
    }
  }

  void TimelineDrawer::drawAction(const Action_t& action, int line_pose, int max_level, int start_time, int end_time, CvFont* font)
  {
    int x_end_pose = line_pose;
    int x_mid_pose = x_end_pose - (int)action.level * SIDE_SPACE;
    int x_start_pose = x_end_pose - (max_level + 1) * SIDE_SPACE;

    int y_start_pose = IMAGE_MARGIN + (action.start.getTime() - start_time) * UNIT_SPACE;
    int y_end_pose = IMAGE_MARGIN + (end_time - start_time) * UNIT_SPACE;
    if(action.end != std::nullopt)
      y_end_pose = IMAGE_MARGIN + (action.end.value().getTime() - start_time) * UNIT_SPACE;
    int y_mid_pose = y_start_pose + EDGE_RADIUS;

    while(used_poses_.find(y_mid_pose) != used_poses_.end())
      y_mid_pose += UNIT_SPACE / 2;
    used_poses_.insert(y_mid_pose);

    // BGR base is 122, 20, 32
    float base_v = 47.8;
    float step = (100 - base_v) / (float)(max_level - 1);
    auto color = scalarHSV2BGR(353. / 360. * 255., 83.6 / 100. * 255., (base_v + (step * (float)(action.level - 1))) / 100.f * 255.f);

    if(action.end == std::nullopt)
    {
      cvLine(image_, cvPoint(x_end_pose, y_start_pose),
             cvPoint(x_mid_pose + EDGE_RADIUS, y_start_pose),
             color, 4);

      cvLine(image_, cvPoint(x_mid_pose, y_start_pose + EDGE_RADIUS),
             cvPoint(x_mid_pose, y_end_pose),
             color, 4);

      int dot_size = UNIT_SPACE / 4;
      cvLine(image_, cvPoint(x_mid_pose, y_end_pose - dot_size * 4),
             cvPoint(x_mid_pose, y_end_pose - dot_size * 3),
             cvScalar(255, 255, 255), 4, CV_AA);

      cvLine(image_, cvPoint(x_mid_pose, y_end_pose - dot_size * 2),
             cvPoint(x_mid_pose, y_end_pose - dot_size),
             cvScalar(255, 255, 255), 4, CV_AA);

      drawElipseStart(x_mid_pose, y_start_pose, color);

      cvLine(image_, cvPoint(x_start_pose, y_mid_pose),
             cvPoint(x_mid_pose, y_mid_pose),
             color, 4);
    }
    else if(action.start.getTime() != action.end.value().getTime())
    {
      cvLine(image_, cvPoint(x_end_pose, y_end_pose),
             cvPoint(x_mid_pose + EDGE_RADIUS, y_end_pose),
             color, 4);

      cvLine(image_, cvPoint(x_end_pose, y_start_pose),
             cvPoint(x_mid_pose + EDGE_RADIUS, y_start_pose),
             color, 4);

      cvLine(image_, cvPoint(x_mid_pose, y_start_pose + EDGE_RADIUS),
             cvPoint(x_mid_pose, y_end_pose - EDGE_RADIUS),
             color, 4);

      drawElipseStart(x_mid_pose, y_start_pose, color);
      drawElipseEnd(x_mid_pose, y_end_pose, color);

      cvLine(image_, cvPoint(x_start_pose, y_mid_pose),
             cvPoint(x_mid_pose, y_mid_pose),
             color, 4);
    }
    else
    {
      cvLine(image_, cvPoint(x_end_pose, y_end_pose),
             cvPoint(x_mid_pose, y_end_pose),
             color, 4);

      y_mid_pose = y_start_pose;
      cvLine(image_, cvPoint(x_start_pose, y_mid_pose),
             cvPoint(x_mid_pose, y_mid_pose),
             color, 4);
    }

    if(action.start.isInstantaneous() == false)
    {
      int y_soft_start_pose = IMAGE_MARGIN + (action.start.getTimeStart() - start_time) * UNIT_SPACE;
      int y_soft_end_pose = IMAGE_MARGIN + (action.start.getTimeEnd() - start_time) * UNIT_SPACE;

      cvLine(image_, cvPoint(line_pose - 4, y_soft_start_pose),
             cvPoint(line_pose - 4, y_soft_end_pose),
             cvScalar(114, 102, 204), 8);
    }

    if(action.end != std::nullopt)
      if(action.end.value().isInstantaneous() == false)
      {
        int y_soft_start_pose = IMAGE_MARGIN + (action.end.value().getTimeStart() - start_time) * UNIT_SPACE;
        int y_soft_end_pose = IMAGE_MARGIN + (action.end.value().getTimeEnd() - start_time) * UNIT_SPACE;

        cvLine(image_, cvPoint(line_pose - 4, y_soft_start_pose),
               cvPoint(line_pose - 4, y_soft_end_pose),
               cvScalar(114, 102, 204), 8);
      }

    cvPutText(image_, action.name.c_str(), cvPoint(x_start_pose - getTextSize(action.name, font) - 2, y_mid_pose), font, color);
  }

  void TimelineDrawer::drawEvent(const Fact_t& event, int line_pose, int start_time, CvFont* font)
  {
    int x_start_pose = line_pose;
    int x_end_pose = x_start_pose + SIDE_SPACE;

    int y_pose = IMAGE_MARGIN + (event.time_point.getTime() - start_time) * UNIT_SPACE;

    cvLine(image_, cvPoint(x_start_pose, y_pose),
           cvPoint(x_end_pose, y_pose),
           cvScalar(89, 26, 16), 4);

    if(event.time_point.isInstantaneous() == false)
    {
      int y_soft_start_pose = IMAGE_MARGIN + (event.time_point.getTimeStart() - start_time) * UNIT_SPACE;
      int y_soft_end_pose = IMAGE_MARGIN + (event.time_point.getTimeEnd() - start_time) * UNIT_SPACE;

      cvLine(image_, cvPoint(line_pose + 4, y_soft_start_pose),
             cvPoint(line_pose + 4, y_soft_end_pose),
             cvScalar(149, 86, 86), 8);
    }

    cvPutText(image_, event.data.c_str(), cvPoint(x_end_pose + 2, y_pose), font,
              cvScalar(89, 26, 16));
  }

  int TimelineDrawer::getTextSize(const std::string& txt, CvFont* font)
  {
    CvSize size;
    int baseline = 0;
    cvGetTextSize(txt.c_str(), font, &size, &baseline);
    return size.width;
  }

  void TimelineDrawer::drawElipseStart(int x, int y, CvScalar color)
  {
    cvEllipse(image_, cvPoint(x + EDGE_RADIUS, y + EDGE_RADIUS), cvSize(EDGE_RADIUS, EDGE_RADIUS), 180, 0, 90, color, 4);
  }

  void TimelineDrawer::drawElipseEnd(int x, int y, CvScalar color)
  {
    cvEllipse(image_, cvPoint(x + EDGE_RADIUS, y - EDGE_RADIUS), cvSize(EDGE_RADIUS, EDGE_RADIUS), 90, 0, 90, color, 4);
  }

  CvScalar TimelineDrawer::scalarHSV2BGR(float H, float S, float V)
  {
    cv::Mat rgb;
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(H, S, V));
    cvtColor(hsv, rgb, CV_HSV2BGR);
    return cvScalar(rgb.data[0], rgb.data[1], rgb.data[2]);
  }

} // namespace mementar
