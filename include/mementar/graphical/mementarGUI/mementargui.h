#ifndef MEMENTAR_MEMENTARGUI_H
#define MEMENTAR_MEMENTARGUI_H

#include <QMainWindow>
#include <QTextCursor>
#include <string>
#include <vector>

#include "include/mementar/graphical/mementarGUI/CallBackTimer.h"
#include "include/mementar/graphical/mementarGUI/QCheckBoxExtended.h"
#include "mementar/API/mementar/TimelinesManipulator.h"
#include "mementar/compat/ros.h"

namespace Ui { // NOLINT
  class MementarGUI;
}

class MementarGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit MementarGUI(QWidget* parent = nullptr);
  ~MementarGUI() override;

  void init();
  void wait();
  void start();

private:
  Ui::MementarGUI* ui_;

  mementar::TimelinesManipulator timelines_;
  mementar::TimelineManipulator* timeline_;
  bool multi_usage_;

  std::string feeder_notifications_;

  int time_source_;

  // todo: can't make it atomic because its not trivially copiable
  mementar::compat::MementarTimestamp current_time_;

  CallBackTimer timer_;

  void displayInstancesList();
  void displayErrorInfo(const std::string& text);

  std::string vector2string(const std::vector<std::string>& vect);
  std::string vector2html(const std::vector<std::string>& vect);

  void updateTime();

public slots:
  void actionButtonHoverEnterSlot();
  void actionButtonHoverLeaveSlot();
  void factButtonHoverEnterSlot();
  void factButtonHoverLeaveSlot();

  void actionButtonClickedSlot();
  void factButtonClickedSlot();

  void nameEditingFinishedSlot();
  void currentTabChangedSlot(int tab_id);

  void displayInstancesListSlot();
  void addInstanceSlot();
  void deleteInstanceSlot();
  void saveInstanceSlot();
  void drawInstanceSlot();
  void instanceNameAddDelChangedSlot(const QString& text);
  void instanceNameChangedSlot(const QString& text);
  void timesourceChangedSlot(int index);
  void currentTimeEditingFinishedSlot();

  void feederCallback(const std::string& msg);
  void feederAddSlot();
  void feederDelSlot();
  void feederCommitSlot();
  void feederCheckoutSlot();

  bool updateTimelinePtr();

signals:
  void feederSetHtmlSignal(QString t1);
  void feederScrollSignal(QString t1);
  void setTimeSignal(QString);
};

#endif // MEMENTAR_MEMENTARGUI_H
