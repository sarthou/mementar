#ifndef MEMENTAR_MEMENTARGUI_H
#define MEMENTAR_MEMENTARGUI_H

#include <QMainWindow>
#include "include/mementar/graphical/mementarGUI/QCheckBoxExtended.h"
#include "include/mementar/graphical/mementarGUI/CallBackTimer.h"
#include <QTextCursor>

#include "mementar/compat/ros.h"
#include "mementar/API/mementar/TimelineManipulator.h"

#include <vector>
#include <string>

namespace Ui {
class mementarGUI;
}

class mementarGUI : public QMainWindow
{
    Q_OBJECT

public:
  explicit mementarGUI(QWidget *parent = 0);
  ~mementarGUI();

  void init();
  void wait();
  void start();

private:
  Ui::mementarGUI* ui;

  mementar::TimelineManipulator meme_;

  std::map<std::string, std::unique_ptr<mementar::compat::onto_ros::Publisher<mementar::compat::StampedString>>> facts_publishers_;
  std::map<std::string, std::unique_ptr<mementar::compat::onto_ros::Publisher<mementar::compat::MementarAction>>> actions_publishers_;
  std::map<std::string, std::unique_ptr<mementar::compat::onto_ros::Subscriber<std_msgs_compat::String>>> feeder_notifications_subs_;
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
  void currentTabChangedSlot(int);

  void displayInstancesListSlot();
  void addInstanceSlot();
  void deleteInstanceSlot();
  void saveInstanceSlot();
  void drawInstanceSlot();
  void InstanceNameAddDelChangedSlot(const QString&);
  void InstanceNameChangedSlot(const QString&);
  void timesourceChangedSlot(int index);
  void currentTimeEditingFinishedSlot();

  void feederCallback(const std_msgs_compat::String& msg);
  void feederAddSlot();
  void feederDelSlot();
  void feederCommitSlot();
  void feederCheckoutSlot();
  void createPublisher(const std::string& onto_ns);

signals:
  void feederSetHtmlSignal(QString);
  void setTimeSignal(QString);
};

#endif // MEMENTAR_MEMENTARGUI_H
