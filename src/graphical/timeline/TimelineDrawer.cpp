#include "mementar/graphical/timeline/TimelineDrawer.h"

#define UNIT_SPACE 35
#define SIDE_SPACE 120
#define EDGE_RADIUS 15
#define TEXT_WIDTH 350
#define MARK_WIDTH 10
#define MARGIN 25

namespace mementar {

bool TimelineDrawer::draw(const std::string& file_name, Timeline* timeline)
{
  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 0, 0.5);

  ActionReader actions_reader_;
  actions_reader_.read(&timeline->facts, &font);
  FactReader facts_reader_;
  facts_reader_.read(&timeline->facts, &font);

  if(timeline->facts.getTimeline()->getFirst() == nullptr)
    return false;

  size_t start = timeline->facts.getTimeline()->getFirst()->getKey();
  size_t end = timeline->facts.getTimeline()->getLast()->getKey();
  size_t width = (actions_reader_.max_level_ + 1) * SIDE_SPACE + actions_reader_.max_text_size_ + MARGIN * 3 + SIDE_SPACE + facts_reader_.max_text_size_;
  size_t height = (end - start) * UNIT_SPACE + MARGIN * 2;

  std::cout << "image size = " << width << " : " << height << std::endl;
  image_ = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  cvSet(image_, cvScalar(255,255,255));

  size_t line_pose = (actions_reader_.max_level_ + 1) * SIDE_SPACE + actions_reader_.max_text_size_ + MARGIN * 2;

  drawVector(start, end, line_pose, &font);

  std::cout << "---ACTIONS----" << std::endl;
  for(auto act : actions_reader_.actions_)
    drawAction(act.second, line_pose, actions_reader_.max_level_, start, &font);

  std::cout << "---FACTS----" << std::endl;
  for(auto evt : facts_reader_.facts)
    drawEvent(evt, line_pose, start, &font);

  if(file_name != "")
  {
    std::cout << "Save image " << file_name << std::endl;

    if((height != 1) && (width != 1))
      cv::imwrite(file_name.c_str(), cv::cvarrToMat(image_));

    if(image_ != nullptr)
      cvReleaseImage(&image_);

    return true;
  }
  else
    return false;
}

void TimelineDrawer::drawVector(size_t start, size_t end, size_t pose, CvFont* font)
{
  cvLine(image_, cvPoint(pose, MARGIN),
                 cvPoint(pose, MARGIN + (end - start) * UNIT_SPACE),
                 cvScalar(50,50,50), 2);


  for(size_t i = 0; i < (end - start) + 1; i++)
  {
    cvLine(image_, cvPoint(pose - MARK_WIDTH/2, MARGIN + i * UNIT_SPACE),
                   cvPoint(pose + MARK_WIDTH/2, MARGIN + i * UNIT_SPACE),
                   cvScalar(50,50,50), 2);

    std::string txt_num = std::to_string(start + i);

    cvPutText(image_, txt_num.c_str(), cvPoint(pose - getTextSize(txt_num, font) - 2,MARGIN + i * UNIT_SPACE - 2), font,
           cvScalar(50,50,50));
  }
}

void TimelineDrawer::drawAction(const action_t& action, size_t line_pose, size_t max_level, size_t start_time, CvFont* font)
{
  size_t x_end_pose = line_pose;
  size_t x_mid_pose = x_end_pose - action.level * SIDE_SPACE;
  size_t x_start_pose = x_end_pose - (max_level + 1) * SIDE_SPACE;

  size_t y_start_pose = MARGIN + (action.start.getTime() - start_time) * UNIT_SPACE;
  size_t y_end_pose = MARGIN + (action.end.value().getTime() - start_time) * UNIT_SPACE;
  size_t y_mid_pose = y_start_pose + EDGE_RADIUS;

  if(action.start.getTime() != action.end.value().getTime())
  {
    cvLine(image_, cvPoint(x_end_pose, y_end_pose),
                       cvPoint(x_mid_pose + EDGE_RADIUS, y_end_pose),
                       cvScalar(32, 20, 122), 4);

    cvLine(image_, cvPoint(x_end_pose, y_start_pose),
                      cvPoint(x_mid_pose + EDGE_RADIUS, y_start_pose),
                      cvScalar(32, 20, 122), 4);

    cvLine(image_, cvPoint(x_mid_pose, y_start_pose + EDGE_RADIUS),
                      cvPoint(x_mid_pose, y_end_pose - EDGE_RADIUS),
                      cvScalar(32, 20, 122), 4);

    drawElipseStart(x_mid_pose, y_start_pose);
    drawElipseEnd(x_mid_pose, y_end_pose);

    cvLine(image_, cvPoint(x_start_pose, y_mid_pose),
                      cvPoint(x_mid_pose, y_mid_pose),
                      cvScalar(32, 20, 122), 4);
  }
  else
  {
    cvLine(image_, cvPoint(x_end_pose, y_end_pose),
                       cvPoint(x_mid_pose, y_end_pose),
                       cvScalar(32, 20, 122), 4);

    y_mid_pose = y_start_pose;
     cvLine(image_, cvPoint(x_start_pose, y_mid_pose),
                       cvPoint(x_mid_pose, y_mid_pose),
                       cvScalar(32, 20, 122), 4);
  }

  if(action.start.isInstantaneous() == false)
  {
    size_t y_soft_start_pose = MARGIN + (action.start.getTimeStart() - start_time) * UNIT_SPACE;
    size_t y_soft_end_pose = MARGIN + (action.start.getTimeEnd() - start_time) * UNIT_SPACE;

    cvLine(image_, cvPoint(line_pose - 4, y_soft_start_pose),
                   cvPoint(line_pose - 4, y_soft_end_pose),
                   cvScalar(114,102,204), 8);
  }

  if(action.end.value().isInstantaneous() == false)
  {
    size_t y_soft_start_pose = MARGIN + (action.end.value().getTimeStart() - start_time) * UNIT_SPACE;
    size_t y_soft_end_pose = MARGIN + (action.end.value().getTimeEnd() - start_time) * UNIT_SPACE;

    cvLine(image_, cvPoint(line_pose - 4, y_soft_start_pose),
                   cvPoint(line_pose - 4, y_soft_end_pose),
                   cvScalar(114,102,204), 8);
  }

  cvPutText(image_, action.name.c_str(), cvPoint(x_start_pose - getTextSize(action.name, font) - 2, y_mid_pose), font,
         cvScalar(32, 20, 122));
}

void TimelineDrawer::drawEvent(const fact_t& event, size_t line_pose, size_t start_time, CvFont* font)
{
  size_t x_start_pose = line_pose;
  size_t x_end_pose = x_start_pose + SIDE_SPACE;

  size_t y_pose = MARGIN + (event.time_point.getTime() - start_time) * UNIT_SPACE;

  cvLine(image_, cvPoint(x_start_pose, y_pose),
                 cvPoint(x_end_pose, y_pose),
                 cvScalar(89, 26, 16), 4);

   if(event.time_point.isInstantaneous() == false)
   {
     size_t y_soft_start_pose = MARGIN + (event.time_point.getTimeStart() - start_time) * UNIT_SPACE;
     size_t y_soft_end_pose = MARGIN + (event.time_point.getTimeEnd() - start_time) * UNIT_SPACE;

     cvLine(image_, cvPoint(line_pose + 4, y_soft_start_pose),
                    cvPoint(line_pose + 4, y_soft_end_pose),
                    cvScalar(149,86,86), 8);
   }


  cvPutText(image_, event.data.c_str(), cvPoint(x_end_pose + 2, y_pose), font,
         cvScalar(89, 26, 16));
}

size_t TimelineDrawer::getTextSize(const std::string& txt, CvFont* font)
{
  CvSize size;
  int baseline;
  cvGetTextSize(txt.c_str(), font, &size, &baseline);
  return size.width;
}

void TimelineDrawer::drawElipseStart(size_t x, size_t y)
{
  cvEllipse(image_, cvPoint(x+EDGE_RADIUS,y+EDGE_RADIUS), cvSize(EDGE_RADIUS, EDGE_RADIUS), 180, 0, 90, cvScalar(32, 20, 122), 4);
}

void TimelineDrawer::drawElipseEnd(size_t x, size_t y)
{
  cvEllipse(image_, cvPoint(x+EDGE_RADIUS,y-EDGE_RADIUS), cvSize(EDGE_RADIUS, EDGE_RADIUS), 90, 0, 90, cvScalar(32, 20, 122), 4);
}

} // namespace mementar
