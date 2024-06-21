#include "mementar/graphical/mementarGUI/mementargui.h"

#include <cstdio>
#include <memory>
#include <regex>
#include <string>
#include <sys/time.h>
#include <vector>

#include "mementar/compat/ros.h"
#include "mementar/graphical/mementarGUI/QLineEditExtended.h"
#include "mementar/graphical/mementarGUI/QPushButtonExtended.h"
#include "qmainwindow.h"
#include "qnamespace.h"
#include "qobject.h"
#include "qobjectdefs.h"
#include "qpushbutton.h"
#include "qtextcursor.h"
#include "qwidget.h"
#include "ui_mementargui.h"

#define QUEU_SIZE 1000

mementarGUI::mementarGUI(QWidget* parent) : QMainWindow(parent), ui_(new Ui::mementarGUI)
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

  QObject::connect(ui_->manager_refresh_button, &QPushButton::clicked, this, &mementarGUI::displayInstancesListSlot);
  QObject::connect(ui_->manager_add_instance_button, &QPushButton::clicked, this, &mementarGUI::addInstanceSlot);
  QObject::connect(ui_->manager_delete_instance_button, &QPushButton::clicked, this, &mementarGUI::deleteInstanceSlot);
  QObject::connect(ui_->manager_save_button, &QPushButton::clicked, this, &mementarGUI::saveInstanceSlot);
  QObject::connect(ui_->manager_draw_button, &QPushButton::clicked, this, &mementarGUI::drawInstanceSlot);

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
  QObject::connect(this, SIGNAL(setTimeSignal(QString)), ui_->static_current_time_editline, SLOT(setText(QString)), Qt::BlockingQueuedConnection);
  QObject::connect(ui_->static_current_time_editline, SIGNAL(editingFinished()), this, SLOT(currentTimeEditingFinishedSlot()));
}

mementarGUI::~mementarGUI()
{
  delete ui_;
}

void mementarGUI::init()
{
  timesourceChangedSlot(0);

  facts_publishers_["_"] = std::make_unique<mementar::compat::onto_ros::Publisher<mementar::compat::StampedString>>("/mementar/insert_fact_stamped", QUEU_SIZE);
  actions_publishers_["_"] = std::make_unique<mementar::compat::onto_ros::Publisher<mementar::compat::MementarAction>>("/mementar/insert_action", QUEU_SIZE);
  feeder_notifications_subs_["_"] = std::make_unique<mementar::compat::onto_ros::Subscriber<std_msgs_compat::String>>("mementar/feeder_notifications", QUEU_SIZE, &mementarGUI::feederCallback, this);
}

void mementarGUI::wait()
{
  QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                 "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                 "p, li { white-space: pre-wrap; }"
                 "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                 "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">Wainting for </span><span style=\" font-size:12pt; font-weight:600; color:#a40000;\">mementar</span></p></body></html>";
  ui_->static_info_area->setHtml(html);
}

void mementarGUI::start()
{
  QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                 "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                 "p, li { white-space: pre-wrap; }"
                 "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                 "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600; color:#4e9a06;\">Mementar</span><span style=\" font-size:12pt; color:#4e9a06;\"> detected</span></p></body></html>";
  ui_->static_info_area->setHtml(html);
}

void mementarGUI::actionButtonHoverEnterSlot()
{
  ui_->action_description_textedit->setText(dynamic_cast<QWidget*>(sender())->whatsThis());
}

void mementarGUI::actionButtonHoverLeaveSlot()
{
  ui_->action_description_textedit->setText("");
}

void mementarGUI::factButtonHoverEnterSlot()
{
  ui_->fact_description_textedit->setText(dynamic_cast<QWidget*>(sender())->whatsThis());
}

void mementarGUI::factButtonHoverLeaveSlot()
{
  ui_->fact_description_textedit->setText("");
}

void mementarGUI::actionButtonClickedSlot()
{
  auto service_name = ui_->static_instance_name_editline->text().toStdString();

  mementar::ActionClient client(service_name);

  auto q_text = dynamic_cast<QPushButtonExtended*>(sender())->text();
  auto q_param = ui_->action_parameter_editline->text();

  QString text = q_text + " : " + ui_->action_parameter_editline->text();
  ui_->action_description_textedit->setText(text);

  auto response = client.call(q_text.toStdString(), q_param.toStdString());

  [&](auto response) {
    if(client.getErrorCode() == -1)
    {
      displayErrorInfo(service_name + " client call failed");
      return;
    }

    start();
    std::string res;

    if(response->values.size())
    {
      res = vector2string(response->values);
    }
    else
    {
      if(response->time_value.seconds != 0)
      {
        res = std::to_string(response->time_value.seconds);
      }
    }

    ui_->static_result_editext->setText(QString::fromStdString(res));
  }(mementar::compat::onto_ros::getServicePointer(response));
}

void mementarGUI::factButtonClickedSlot()
{
  std::string service_name = ui_->static_instance_name_editline->text().toStdString();

  mementar::FactClient client(service_name);

  auto q_text = dynamic_cast<QPushButtonExtended*>(sender())->text();
  auto q_param = ui_->fact_parameter_editline->text();

  QString text = q_text + " : " + ui_->fact_parameter_editline->text();
  ui_->fact_description_textedit->setText(text);

  auto response = client.call(q_text.toStdString(), q_param.toStdString());

  [&](auto response) {
    if(client.getErrorCode() == -1)
    {
      displayErrorInfo(service_name + " client call failed");
      return;
    }

    start();
    std::string res;

    if(response->values.size())
    {
      res = vector2string(response->values);
    }
    else
    {
      if(response->time_value.seconds != 0)
      {
        res = std::to_string(response->time_value.seconds);
      }
    }

    ui_->static_result_editext->setText(QString::fromStdString(res));
  }(mementar::compat::onto_ros::getServicePointer(response));
}

void mementarGUI::nameEditingFinishedSlot()
{
}

void mementarGUI::displayErrorInfo(const std::string& text)
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

void mementarGUI::displayInstancesList()
{
  auto values = meme_.manager_.list();

  std::string html;

  if(meme_.manager_.getErrorCode() == -1)
  {
    html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
           "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
           "p, li { white-space: pre-wrap; }"
           "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
           "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">mementar is not running in multi mode.</span></p></body></html>";
  }
  else
  {
    std::string text = vector2html(values);
    html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
           "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
           "p, li { white-space: pre-wrap; }"
           "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
           "<p align=\"left\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; \">" +
           text + "</span></p></body></html>";
  }

  ui_->manager_instances_list_edittext->setHtml(QString::fromStdString(html));
}

void mementarGUI::displayInstancesListSlot()
{
  displayInstancesList();
}

std::string mementarGUI::vector2string(const std::vector<std::string>& vect)
{
  std::string res;
  for(const auto& v : vect)
    res += v + "\n";
  return res;
}

std::string mementarGUI::vector2html(const std::vector<std::string>& vect)
{
  std::string res;
  for(const auto& v : vect)
    res += " - " + v + "<br>";
  return res;
}

void mementarGUI::updateTime()
{
  if(time_source_ == 0)
  {
    auto time = mementar::compat::onto_ros::Node::get().current_time();
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

void mementarGUI::currentTabChangedSlot(int tab_id)
{
  if(tab_id == 3)
    displayInstancesList();
}

void mementarGUI::addInstanceSlot()
{
  bool do_copy = false;

  std::string param = ui_->manager_instance_name_editline->text().toStdString();

  std::regex base_regex("(.*)=(.*)");
  std::smatch base_match;
  if(std::regex_match(param, base_match, base_regex))
  {
    if(base_match.size() == 3)
    {
      do_copy = true;

      if(facts_publishers_.find(base_match[1].str()) == facts_publishers_.end())
      {
        facts_publishers_[base_match[1].str()] = std::make_unique<mementar::compat::onto_ros::Publisher<mementar::compat::StampedString>>(
          "mementar/insert_fact_stamped/" + base_match[1].str(), QUEU_SIZE);

        actions_publishers_[base_match[1].str()] = std::make_unique<mementar::compat::onto_ros::Publisher<mementar::compat::MementarAction>>(
          "mementar/insert_action/" + base_match[1].str(), QUEU_SIZE);
      }

      if(feeder_notifications_subs_.find(base_match[1].str()) == feeder_notifications_subs_.end())
        feeder_notifications_subs_[base_match[1].str()] = std::make_unique<mementar::compat::onto_ros::Subscriber<std_msgs_compat::String>>(
          "mementar/feeder_notifications", QUEU_SIZE, &mementarGUI::feederCallback, this);
    }
  }
  else
  {
    if(facts_publishers_.find(param) == facts_publishers_.end())
    {
      facts_publishers_[param] = std::make_unique<mementar::compat::onto_ros::Publisher<mementar::compat::StampedString>>(
        "mementar/insert_fact_stamped/" + param, QUEU_SIZE);
      actions_publishers_[param] = std::make_unique<mementar::compat::onto_ros::Publisher<mementar::compat::MementarAction>>(
        "mementar/insert_action/" + param, QUEU_SIZE);
    }

    if(feeder_notifications_subs_.find(param) == feeder_notifications_subs_.end())
      feeder_notifications_subs_[param] = std::make_unique<mementar::compat::onto_ros::Subscriber<std_msgs_compat::String>>(
        "mementar/feeder_notifications", QUEU_SIZE, &mementarGUI::feederCallback, this);
  }

  auto code = do_copy ? meme_.manager_.copy(param) : meme_.manager_.add(param);

  if(meme_.manager_.getErrorCode() == -1)
  {
    displayErrorInfo("mementar/manage client call failed");
    return;
  }

  switch(code)
  {
  case 1:
  {
    ui_->static_result_editext->setText(QString::fromStdString("fail to stop " + param + " : please retry"));
    break;
  }
  case 4:
  {
    ui_->static_result_editext->setText(QString::fromStdString(param + " already created"));
    break;
  }
  default:
  {
    ui_->static_result_editext->setText(QString::fromStdString(""));
    break;
  }
  }

  displayInstancesList();
}

void mementarGUI::deleteInstanceSlot()
{
  auto param = ui_->manager_instance_name_editline->text().toStdString();
  auto code = meme_.manager_.del(ui_->manager_instance_name_editline->text().toStdString());

  if(meme_.manager_.getErrorCode() == -1)
  {
    displayErrorInfo("mementar/manage client call failed");
    return;
  }

  start();

  if(code == 4)
  {
    ui_->static_result_editext->setText(QString::fromStdString("Instance \'" + param + "\' don't exist"));
  }
  else
  {
    ui_->static_result_editext->setText(QString::fromStdString(""));
  }

  displayInstancesList();
}

void mementarGUI::saveInstanceSlot()
{
  auto service_name = ui_->manager_instance_name_editline->text().toStdString();
  auto param = ui_->manager_save_path_editline->text().toStdString();

  mementar::InstanceManagerClient client(service_name);

  auto code = client.save(ui_->manager_save_path_editline->text().toStdString());

  if(client.getErrorCode() == -1)
  {
    displayErrorInfo("mementar/manage_instance client call failed");
    return;
  }

  if(code == 4)
  {
    ui_->static_result_editext->setText(QString::fromStdString("path \'" + param + "\' don't exist"));
  }
  else
  {
    ui_->static_result_editext->setText(QString::fromStdString(""));
  }
}

void mementarGUI::drawInstanceSlot()
{
  auto service_name = ui_->manager_instance_name_editline->text().toStdString();
  auto param = ui_->manager_draw_path_editline->text().toStdString();

  mementar::InstanceManagerClient client(service_name);

  auto code = client.draw(ui_->manager_save_path_editline->text().toStdString());

  if(client.getErrorCode() == -1)
  {
    displayErrorInfo("mementar/manage_instance client call failed");
    return;
  }

  if(code == 4)
  {
    ui_->static_result_editext->setText(QString::fromStdString("path \'" + param + "\' don't exist"));
  }
  else
  {
    ui_->static_result_editext->setText(QString::fromStdString(""));
  }
}

void mementarGUI::instanceNameAddDelChangedSlot(const QString& text)
{
  if(ui_->static_instance_name_editline->text() != text)
    ui_->static_instance_name_editline->setText(text);
}

void mementarGUI::instanceNameChangedSlot(const QString& text)
{
  if(ui_->manager_instance_name_editline->text() != text)
    ui_->manager_instance_name_editline->setText(text);
}

void mementarGUI::timesourceChangedSlot(int index)
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

void mementarGUI::currentTimeEditingFinishedSlot()
{
  if(time_source_ == 2)
  {
    std::string time_str = ui_->static_current_time_editline->text().toStdString();
    int time_int = 0;
    if(sscanf(time_str.c_str(), "%d", &time_int) == 1)
    {
      current_time_.seconds = time_int;
      current_time_.nanoseconds = 0;
      // current_time_.store(mementar::compat::onto_ros::Time(time_int, 0), std::memory_order_release);

      ui_->static_result_editext->setText(QString::fromStdString(""));
    }
    else
    {
      ui_->static_result_editext->setText(
        QString::fromStdString("impossible to convert " + time_str + " to integer"));
    }
  }
}

void mementarGUI::feederCallback(const std_msgs_compat::String& msg)
{
  feeder_notifications_ += "<p>-" + msg.data + "</p>";

  std::string html =
    "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
    "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
    "p, li { whicommitte-space: pre-wrap; }"
    "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
    "<p align=\"left\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; \">" +
    feeder_notifications_ + "<br></span></p></body></html>";

  ui_->feeder_info_edittext->moveCursor(QTextCursor::End);
  feederSetHtmlSignal(QString::fromStdString(html));
  ui_->feeder_info_edittext->ensureCursorVisible();
}

void mementarGUI::feederAddSlot()
{
  std::string subject = ui_->feeder_subject_editline->text().toStdString();
  std::string predicat = ui_->feeder_property_editline->text().toStdString();
  std::string object = ui_->feeder_object_editline->text().toStdString();

  if((subject.empty()) && (predicat.empty()) && (object.empty()))
  {
    return;
  }
  else
  {
    std::string instance_ns = ui_->static_instance_name_editline->text().toStdString();
    if(instance_ns.empty())
      instance_ns = "_";
    createPublisher(instance_ns);
    if((predicat.empty()) && (object.empty()))
    {
      mementar::compat::MementarAction msg;
      msg.name = subject;
      msg.start_stamp = current_time_;
      msg.end_stamp.seconds = 0;
      msg.end_stamp.nanoseconds = 0;
      actions_publishers_[instance_ns]->publish(msg);
    }
    else
    {
      mementar::compat::StampedString msg;
      msg.data = "[ADD]" + subject + "|" + predicat + "|" + object;
      msg.stamp = current_time_;
      facts_publishers_[instance_ns]->publish(msg);
    }
  }
}

void mementarGUI::feederDelSlot()
{
  std::string subject = ui_->feeder_subject_editline->text().toStdString();
  std::string predicat = ui_->feeder_property_editline->text().toStdString();
  std::string object = ui_->feeder_object_editline->text().toStdString();

  if((subject.empty()) && (predicat.empty()) && (object.empty()))
  {
    return;
  }
  else
  {
    std::string instance_ns = ui_->static_instance_name_editline->text().toStdString();
    if(instance_ns.empty())
      instance_ns = "_";
    createPublisher(instance_ns);
    if((predicat.empty()) && (object.empty()))
    {
      mementar::compat::MementarAction msg;
      msg.name = subject;
      msg.start_stamp.seconds = 0;
      msg.start_stamp.nanoseconds = 0;
      msg.end_stamp = current_time_;
      actions_publishers_[instance_ns]->publish(msg);
    }
    else
    {
      mementar::compat::StampedString msg;
      msg.data = "[DEL]" + subject + "|" + predicat + "|" + object;
      msg.stamp = current_time_;
      facts_publishers_[instance_ns]->publish(msg);
    }
  }
}

void mementarGUI::feederCommitSlot()
{
  mementar::compat::StampedString msg;
  msg.data = "[commit]" + ui_->feeder_commit_name_editline->text().toStdString() + "|";
  std::string instance_ns = ui_->static_instance_name_editline->text().toStdString();
  if(instance_ns.empty())
    instance_ns = "_";
  createPublisher(instance_ns);
  facts_publishers_[instance_ns]->publish(msg);
}

void mementarGUI::feederCheckoutSlot()
{
  mementar::compat::StampedString msg;
  msg.data = "[checkout]" + ui_->feeder_commit_name_editline->text().toStdString() + "|";
  std::string instance_ns = ui_->static_instance_name_editline->text().toStdString();
  if(instance_ns.empty())
    instance_ns = "_";
  createPublisher(instance_ns);
  facts_publishers_[instance_ns]->publish(msg);
}

void mementarGUI::createPublisher(const std::string& instance_ns)
{
  if(facts_publishers_.find(instance_ns) == facts_publishers_.end())
  {
    facts_publishers_[instance_ns] = std::make_unique<mementar::compat::onto_ros::Publisher<mementar::compat::StampedString>>(
      "mementar/insert_fact_stamped/" + instance_ns, QUEU_SIZE);

    while(mementar::compat::onto_ros::Node::ok() && (facts_publishers_[instance_ns]->getNumSubscribers() == 0))
    {
      // todo
      // ros::spinOnce();
    }
  }

  if(actions_publishers_.find(instance_ns) == actions_publishers_.end())
  {
    actions_publishers_[instance_ns] = std::make_unique<mementar::compat::onto_ros::Publisher<mementar::compat::MementarAction>>(
      "mementar/insert_action/" + instance_ns, QUEU_SIZE);

    while(mementar::compat::onto_ros::Node::ok() && (actions_publishers_[instance_ns]->getNumSubscribers() == 0))
    {
      // todo
      // ros::spinOnce();
    }
  }
}
