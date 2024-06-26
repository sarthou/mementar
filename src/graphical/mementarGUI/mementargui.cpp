#include "mementar/graphical/mementarGUI/mementargui.h"

#include <algorithm>
#include <bits/types/struct_timeval.h>
#include <cstdio>
#include <regex>
#include <string>
#include <sys/time.h>
#include <vector>

#include "mementar/API/mementar/Fact.h"
#include "mementar/API/mementar/TimelineManipulator.h"
#include "mementar/API/mementar/TimelinesManipulator.h"
#include "mementar/compat/ros.h"
#include "mementar/graphical/mementarGUI/QLineEditExtended.h"
#include "mementar/graphical/mementarGUI/QPushButtonExtended.h"
#include "qmainwindow.h"
#include "qnamespace.h"
#include "qobject.h"
#include "qobjectdefs.h"
#include "qpushbutton.h"
#include "qwidget.h"
#include "ui_mementargui.h"

#define QUEU_SIZE 1000

MementarGUI::MementarGUI(QWidget* parent) : QMainWindow(parent), ui_(new Ui::MementarGUI)
{
  ui_->setupUi(this);

  QObject::connect(ui_->action_exist_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_exist_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));
  QObject::connect(ui_->action_removeAction_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_removeAction_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));
  QObject::connect(ui_->action_getPending_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_getPending_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));
  QObject::connect(ui_->action_isPending_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_isPending_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));
  QObject::connect(ui_->action_getStartStamp_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_getStartStamp_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));
  QObject::connect(ui_->action_getEndStamp_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_getEndStamp_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));
  QObject::connect(ui_->action_getDuration_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_getDuration_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));
  QObject::connect(ui_->action_getStartFact_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_getStartFact_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));
  QObject::connect(ui_->action_getEndFact_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_getEndFact_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));
  QObject::connect(ui_->action_getFactsDuring_button, SIGNAL(hoverEnter()), this, SLOT(actionButtonHoverEnterSlot()));
  QObject::connect(ui_->action_getFactsDuring_button, SIGNAL(hoverLeave()), this, SLOT(actionButtonHoverLeaveSlot()));

  QObject::connect(ui_->fact_exist_button, SIGNAL(hoverEnter()), this, SLOT(factButtonHoverEnterSlot()));
  QObject::connect(ui_->fact_exist_button, SIGNAL(hoverLeave()), this, SLOT(factButtonHoverLeaveSlot()));
  QObject::connect(ui_->fact_getData_button, SIGNAL(hoverEnter()), this, SLOT(factButtonHoverEnterSlot()));
  QObject::connect(ui_->fact_getData_button, SIGNAL(hoverLeave()), this, SLOT(factButtonHoverLeaveSlot()));
  QObject::connect(ui_->fact_getActionPart_button, SIGNAL(hoverEnter()), this, SLOT(factButtonHoverEnterSlot()));
  QObject::connect(ui_->fact_getActionPart_button, SIGNAL(hoverLeave()), this, SLOT(factButtonHoverLeaveSlot()));
  QObject::connect(ui_->fact_isActionPart_button, SIGNAL(hoverEnter()), this, SLOT(factButtonHoverEnterSlot()));
  QObject::connect(ui_->fact_isActionPart_button, SIGNAL(hoverLeave()), this, SLOT(factButtonHoverLeaveSlot()));
  QObject::connect(ui_->fact_getStamp_button, SIGNAL(hoverEnter()), this, SLOT(factButtonHoverEnterSlot()));
  QObject::connect(ui_->fact_getStamp_button, SIGNAL(hoverLeave()), this, SLOT(factButtonHoverLeaveSlot()));

  QObject::connect(ui_->action_exist_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));
  QObject::connect(ui_->action_getPending_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));
  QObject::connect(ui_->action_removeAction_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));
  QObject::connect(ui_->action_isPending_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));
  QObject::connect(ui_->action_getStartStamp_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));
  QObject::connect(ui_->action_getEndStamp_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));
  QObject::connect(ui_->action_getDuration_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));
  QObject::connect(ui_->action_getStartFact_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));
  QObject::connect(ui_->action_getEndFact_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));
  QObject::connect(ui_->action_getFactsDuring_button, SIGNAL(clicked()), this, SLOT(actionButtonClickedSlot()));

  QObject::connect(ui_->fact_exist_button, SIGNAL(clicked()), this, SLOT(factButtonClickedSlot()));
  QObject::connect(ui_->fact_getData_button, SIGNAL(clicked()), this, SLOT(factButtonClickedSlot()));
  QObject::connect(ui_->fact_getActionPart_button, SIGNAL(clicked()), this, SLOT(factButtonClickedSlot()));
  QObject::connect(ui_->fact_isActionPart_button, SIGNAL(clicked()), this, SLOT(factButtonClickedSlot()));
  QObject::connect(ui_->fact_getStamp_button, SIGNAL(clicked()), this, SLOT(factButtonClickedSlot()));

  QObject::connect(ui_->manager_refresh_button, &QPushButton::clicked, this, &MementarGUI::displayInstancesListSlot);
  QObject::connect(ui_->manager_add_instance_button, &QPushButton::clicked, this, &MementarGUI::addInstanceSlot);
  QObject::connect(ui_->manager_delete_instance_button, &QPushButton::clicked, this, &MementarGUI::deleteInstanceSlot);
  QObject::connect(ui_->manager_save_button, &QPushButton::clicked, this, &MementarGUI::saveInstanceSlot);
  QObject::connect(ui_->manager_draw_button, &QPushButton::clicked, this, &MementarGUI::drawInstanceSlot);

  QObject::connect(ui_->feeder_add_start_button, SIGNAL(clicked()), this, SLOT(feederAddSlot()));
  QObject::connect(ui_->feeder_remove_end_button, SIGNAL(clicked()), this, SLOT(feederDelSlot()));
  QObject::connect(ui_->feeder_commit_button, SIGNAL(clicked()), this, SLOT(feederCommitSlot()));
  QObject::connect(ui_->feeder_checkout_button, SIGNAL(clicked()), this, SLOT(feederCheckoutSlot()));

  QObject::connect(ui_->manager_instance_name_editline, SIGNAL(textChanged(const QString&)), this, SLOT(instanceNameAddDelChangedSlot(const QString&)));
  QObject::connect(ui_->static_instance_name_editline, SIGNAL(textChanged(const QString&)), this, SLOT(instanceNameChangedSlot(const QString&)));
  QObject::connect(ui_->static_instance_name_editline, SIGNAL(editingFinished()), this, SLOT(nameEditingFinishedSlot()));
  QObject::connect(ui_->static_time_source_combobox, SIGNAL(currentIndexChanged(int)), this, SLOT(timesourceChangedSlot(int)));
  QObject::connect(ui_->static_tab_widget, SIGNAL(currentChanged(int)), this, SLOT(currentTabChangedSlot(int)));

  QObject::connect(this, SIGNAL(feederSetHtmlSignal(QString)), ui_->feeder_info_edittext, SLOT(setHtml(QString)), Qt::BlockingQueuedConnection);
  QObject::connect(this, SIGNAL(feederScrollSignal(QString)), ui_->feeder_info_edittext, SLOT(scrollToAnchor(QString)), Qt::BlockingQueuedConnection);
  QObject::connect(this, SIGNAL(setTimeSignal(QString)), ui_->static_current_time_editline, SLOT(setText(QString)), Qt::BlockingQueuedConnection);
  QObject::connect(ui_->static_current_time_editline, SIGNAL(editingFinished()), this, SLOT(currentTimeEditingFinishedSlot()));
}

MementarGUI::~MementarGUI()
{
  delete ui_;
}

void MementarGUI::init()
{
  timesourceChangedSlot(0);

  if(timelines_.waitInit(1) == false)
  {
    timeline_ = new mementar::TimelineManipulator();
    timeline_->action_feeder_.registerFeederNotificationCallback([this](auto msg) { this->feederCallback(msg); });
    timeline_->fact_feeder_.registerFeederNotificationCallback([this](auto msg) { this->feederCallback(msg); });
    multi_usage_ = false;
  }
  else
  {
    timeline_ = nullptr;
    multi_usage_ = true;
  }
}

void MementarGUI::wait()
{
  QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                 "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                 "p, li { white-space: pre-wrap; }"
                 "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                 "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">Wainting for </span><span style=\" font-size:12pt; font-weight:600; color:#a40000;\">mementar</span></p></body></html>";
  ui_->static_info_area->setHtml(html);
}

void MementarGUI::start()
{
  QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                 "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                 "p, li { white-space: pre-wrap; }"
                 "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                 "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600; color:#4e9a06;\">Mementar</span><span style=\" font-size:12pt; color:#4e9a06;\"> detected</span></p></body></html>";
  ui_->static_info_area->setHtml(html);
}

void MementarGUI::actionButtonHoverEnterSlot()
{
  ui_->action_description_textedit->setText(dynamic_cast<QWidget*>(sender())->whatsThis());
}

void MementarGUI::actionButtonHoverLeaveSlot()
{
  ui_->action_description_textedit->setText("");
}

void MementarGUI::factButtonHoverEnterSlot()
{
  ui_->fact_description_textedit->setText(dynamic_cast<QWidget*>(sender())->whatsThis());
}

void MementarGUI::factButtonHoverLeaveSlot()
{
  ui_->fact_description_textedit->setText("");
}

void MementarGUI::actionButtonClickedSlot()
{
  if(ui_->static_instance_name_editline->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("Instance name cannot have the symbol = : \'" + ui_->static_instance_name_editline->text().toStdString() + "\'\n Once an instance copy has been performed, use the new instance name.");
    return;
  }

  const std::string action = dynamic_cast<QPushButtonExtended*>(sender())->text().toStdString();
  const std::string param = ui_->action_parameter_editline->text().toStdString();
  if(updateTimelinePtr() == false)
    return;

  const QString text = dynamic_cast<QPushButtonExtended*>(sender())->text() + " : " + ui_->action_parameter_editline->text();
  ui_->action_description_textedit->setText(text);

  auto response = timeline_->actions_.call(action, param);
  const int err = timeline_->actions_.getErrorCode();
  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    start();
    std::string res;
    if(response.first.empty() == false)
      res = vector2string(response.first);
    else if(response.second.seconds() != 0)
      res = std::to_string(response.second.seconds());
    ui_->static_result_editext->setText(QString::fromStdString(res));
  }
}

void MementarGUI::factButtonClickedSlot()
{
  if(ui_->static_instance_name_editline->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("Instance name cannot have the symbol = : \'" + ui_->static_instance_name_editline->text().toStdString() + "\'\n Once an instance copy has been performed, use the new instance name.");
    return;
  }

  const std::string action = dynamic_cast<QPushButtonExtended*>(sender())->text().toStdString();
  const std::string param = ui_->fact_parameter_editline->text().toStdString();
  if(updateTimelinePtr() == false)
    return;

  const QString text = dynamic_cast<QPushButtonExtended*>(sender())->text() + " : " + ui_->fact_parameter_editline->text();
  ui_->fact_description_textedit->setText(text);

  auto response = timeline_->facts_.call(action, param);
  const int err = timeline_->facts_.getErrorCode();

  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    start();
    std::string res;
    if(response.first.empty() == false)
      res = vector2string(response.first);
    else if(response.second.seconds() != 0)
      res = std::to_string(response.second.seconds());
    ui_->static_result_editext->setText(QString::fromStdString(res));
  }
}

void MementarGUI::nameEditingFinishedSlot()
{
  if(updateTimelinePtr() == false)
    return;
}

void MementarGUI::displayErrorInfo(const std::string& text)
{
  std::string html =
    "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
    "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
    "p, li { white-space: pre-wrap; }"
    "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
    "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">" +
    text + "</span></p></body></html>";
  ui_->static_info_area->setHtml(QString::fromStdString(html));
}

void MementarGUI::displayInstancesList()
{
  auto res_vect = timelines_.list();
  const int err = timelines_.getErrorCode();

  std::string html;

  if(err == -1)
  {
    html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
           "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
           "p, li { white-space: pre-wrap; }"
           "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
           "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">mementar is not running in multi mode.</span></p></body></html>";
  }
  else
  {
    std::string text = vector2html(res_vect);
    html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
           "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
           "p, li { white-space: pre-wrap; }"
           "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
           "<p align=\"left\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; \">" +
           text + "</span></p></body></html>";
  }

  ui_->manager_instances_list_edittext->setHtml(QString::fromStdString(html));
}

void MementarGUI::displayInstancesListSlot()
{
  displayInstancesList();
}

std::string MementarGUI::vector2string(const std::vector<std::string>& vect)
{
  std::string res;
  for(const auto& v : vect)
    res += v + "\n";
  return res;
}

std::string MementarGUI::vector2html(const std::vector<std::string>& vect)
{
  std::string res;
  for(const auto& v : vect)
    res += " - " + v + "<br>";
  return res;
}

void MementarGUI::updateTime() // todo fix time display
{
  if(time_source_ == 0)
  {
    auto time = mementar::compat::mem_ros::Node::get().currentTime();
    current_time_.seconds = time.seconds();
    current_time_.nanoseconds = time.nanoseconds();
    // current_time_.store(ros::Time::now(), std::memory_order_release);
    // setTimeSignal(QString::fromStdString(std::to_string(current_time_.load(std::memory_order_acquire).sec)));
    // ui_->static_current_time_editline->setText();
  }
  else if(time_source_ == 1)
  {
    struct timeval tp;
    gettimeofday(&tp, nullptr);
    // current_time_.store(ros::Time(tp.tv_sec, tp.tv_usec), std::memory_order_release);
    current_time_.seconds = tp.tv_sec;
    current_time_.nanoseconds = tp.tv_usec;
    // setTimeSignal(QString::fromStdString(std::to_string(current_time_.load(std::memory_order_acquire).sec)));
  }

  setTimeSignal(QString::fromStdString(std::to_string(current_time_.seconds)));
}

void MementarGUI::currentTabChangedSlot(int tab_id)
{
  if(tab_id == 3)
    displayInstancesList();
}

void MementarGUI::addInstanceSlot()
{
  std::string param = ui_->manager_instance_name_editline->text().toStdString();
  const std::string& inst_name = param;

  std::regex base_regex("(.*)=(.*)");
  std::smatch base_match;
  if(std::regex_match(param, base_match, base_regex))
  {
    if(base_match.size() == 3)
    {
      ui_->static_result_editext->setText(QString::fromStdString("Copy is not yet supported."));
      return;
      // timelines_.copy(base_match[1].str(), base_match[2].str());
      // inst_name = base_match[1].str();
    }
  }
  else
    timelines_.add(param);

  const int err = timelines_.getErrorCode();
  if(err == -1)
    displayErrorInfo("mementar/manage client call failed");
  else
  {
    start();
    if(err == 4)
      ui_->static_result_editext->setText(QString::fromStdString(param + " already created"));
    else if(err == 1)
      ui_->static_result_editext->setText(QString::fromStdString("fail to stop " + param + " : please retry"));
    else
    {
      ui_->static_result_editext->setText(QString::fromStdString(""));
      timelines_.get(inst_name)->action_feeder_.registerFeederNotificationCallback([this](auto msg) { this->feederCallback(msg); });
      timelines_.get(inst_name)->fact_feeder_.registerFeederNotificationCallback([this](auto msg) { this->feederCallback(msg); });
    }

    displayInstancesList();
  }
}

void MementarGUI::deleteInstanceSlot()
{
  auto param = ui_->manager_instance_name_editline->text().toStdString();
  timelines_.del(param);
  const int err = timelines_.getErrorCode();

  if(err == -1)
  {
    displayErrorInfo("mementar/manage client call failed");
    return;
  }

  start();
  if(err == 4)
    ui_->static_result_editext->setText(QString::fromStdString("Instance \'" + param + "\' don't exist"));
  else
    ui_->static_result_editext->setText(QString::fromStdString(""));
  displayInstancesList();
}

void MementarGUI::saveInstanceSlot()
{
  if(ui_->manager_instance_name_editline->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("Instance name cannot have the symbol = : \'" + ui_->manager_instance_name_editline->text().toStdString() + "\'\n Once an instance copy has been performed, use the new instance name.");
    return;
  }

  if(updateTimelinePtr() == false)
    return;

  auto param = ui_->manager_save_path_editline->text().toStdString();
  timeline_->inst_manager_.save(param);
  const int err = timeline_->inst_manager_.getErrorCode();

  if(err == -1)
  {
    displayErrorInfo("mementar/manage_instance client call failed");
    return;
  }
  else
  {
    if(err == 4)
      ui_->static_result_editext->setText(QString::fromStdString("path \'" + param + "\' don't exist"));
    else
      ui_->static_result_editext->setText(QString::fromStdString(""));
  }
}

void MementarGUI::drawInstanceSlot()
{
  if(ui_->manager_instance_name_editline->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("Instance name cannot have the symbol = : \'" + ui_->manager_instance_name_editline->text().toStdString() + "\'\n Once an instance copy has been performed, use the new instance name.");
    return;
  }

  if(updateTimelinePtr() == false)
    return;

  auto param = ui_->manager_save_path_editline->text().toStdString();
  timeline_->inst_manager_.draw(param);
  const int err = timeline_->inst_manager_.getErrorCode();

  if(err == -1)
  {
    displayErrorInfo("mementar/manage_instance client call failed");
    return;
  }
  else
  {
    if(err == 4)
      ui_->static_result_editext->setText(QString::fromStdString("path \'" + param + "\' don't exist"));
    else
      ui_->static_result_editext->setText(QString::fromStdString(""));
  }
}

void MementarGUI::instanceNameAddDelChangedSlot(const QString& text)
{
  if(ui_->static_instance_name_editline->text() != text)
  {
    const size_t equal_pose = text.toStdString().find('=');
    if(equal_pose != std::string::npos)
      ui_->static_instance_name_editline->setText(text.mid(0, (int)equal_pose));
    else
      ui_->static_instance_name_editline->setText(text);
  }
}

void MementarGUI::instanceNameChangedSlot(const QString& text)
{
  if(ui_->manager_instance_name_editline->text() != text)
    ui_->manager_instance_name_editline->setText(text);
}

void MementarGUI::timesourceChangedSlot(int index)
{
  time_source_ = index;
  if(index == 2) // manual
  {
    if(timer_.isRunning())
    {
      timer_.stop();
    }

    ui_->static_current_time_editline->setReadOnly(false);
  }
  else
  {
    ui_->static_current_time_editline->setReadOnly(true);

    if(timer_.isRunning() == false)
    {
      timer_.start(250, [this]() { this->updateTime(); });
    }
  }
}

void MementarGUI::currentTimeEditingFinishedSlot()
{
  if(time_source_ == 2)
  {
    std::string time_str = ui_->static_current_time_editline->text().toStdString();
    int time_int = 0;
    if(sscanf(time_str.c_str(), "%d", &time_int) == 1)
    {
      current_time_.seconds = time_int;
      current_time_.nanoseconds = 0;
      // current_time_.store(mementar::compat::mem_ros::Time(time_int, 0), std::memory_order_release);

      ui_->static_result_editext->setText(QString::fromStdString(""));
    }
    else
    {
      ui_->static_result_editext->setText(
        QString::fromStdString("impossible to convert " + time_str + " to integer"));
    }
  }
}

void MementarGUI::feederCallback(const std::string& msg)
{
  feeder_notifications_ += "<p>-" + msg + "</p>";

  const std::string html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                           "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                           "p, li { whicommitte-space: pre-wrap; }"
                           "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                           "<p align=\"left\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; \">" +
                           feeder_notifications_ + R"(<a name="scrollToMe" href="#scroll"></a> <br></span></p></body></html>)";

  feederSetHtmlSignal(QString::fromStdString(html));
  feederScrollSignal("scrollToMe");
}

void MementarGUI::feederAddSlot()
{
  if(updateTimelinePtr() == false)
    return;

  std::string subject = ui_->feeder_subject_editline->text().toStdString();
  std::string predicat = ui_->feeder_property_editline->text().toStdString();
  std::string object = ui_->feeder_object_editline->text().toStdString();

  if((subject.empty()) && (predicat.empty()) && (object.empty()))
  {
    return;
  }
  else
  {
    if((predicat.empty()) && (object.empty()))
      timeline_->action_feeder_.insert(subject, mementar::compat::mem_ros::Time(current_time_.seconds, current_time_.nanoseconds));
    else
    {
      mementar::Fact fact(subject, predicat, object, true);
      timeline_->fact_feeder_.insert(fact, mementar::compat::mem_ros::Time(current_time_.seconds, current_time_.nanoseconds));
    }
  }
}

void MementarGUI::feederDelSlot()
{
  if(updateTimelinePtr() == false)
    return;

  std::string subject = ui_->feeder_subject_editline->text().toStdString();
  std::string predicat = ui_->feeder_property_editline->text().toStdString();
  std::string object = ui_->feeder_object_editline->text().toStdString();

  if((subject.empty()) && (predicat.empty()) && (object.empty()))
  {
    return;
  }
  else
  {
    if((predicat.empty()) && (object.empty()))
      timeline_->action_feeder_.insertEnd(subject, mementar::compat::mem_ros::Time(current_time_.seconds, current_time_.nanoseconds));
    else
    {
      mementar::Fact fact(subject, predicat, object, false);
      timeline_->fact_feeder_.insert(fact, mementar::compat::mem_ros::Time(current_time_.seconds, current_time_.nanoseconds));
    }
  }
}

void MementarGUI::feederCommitSlot()
{
  if(updateTimelinePtr() == false)
    return;

  // timeline_->fact_feeder_.commit(ui_->feeder_commit_name_editline->text().toStdString());
}

void MementarGUI::feederCheckoutSlot()
{
  if(updateTimelinePtr() == false)
    return;

  // timeline_->fact_feeder_.checkout(ui_->feeder_commit_name_editline->text().toStdString());
}

bool MementarGUI::updateTimelinePtr()
{
  if(multi_usage_ == false)
    return true;

  const std::string instance_name = ui_->static_instance_name_editline->text().toStdString();
  timeline_ = timelines_.get(instance_name);
  if(timeline_ == nullptr)
  {
    auto intances_name = timelines_.list();
    if(std::find(intances_name.begin(), intances_name.end(), instance_name) != intances_name.end())
    {
      timelines_.add(instance_name);
      timeline_ = timelines_.get(instance_name);
      if(timeline_ != nullptr)
        return true;
    }

    if(instance_name.empty() == false)
      displayErrorInfo("Instance " + instance_name + " does not exist");
    return false;
  }
  else
    return true;
}
