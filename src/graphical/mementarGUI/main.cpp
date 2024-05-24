#include "include/mementar/graphical/mementarGUI/DarkStyle.h"
#include "include/mementar/graphical/mementarGUI/mementargui.h"

#include <QApplication>

#include <csignal>
#include <thread>

#include "mementar/compat/ros.h"

void spinThread(bool* run)
{
  mementar::compat::onto_ros::Rate r(100);
  while(*run == true) {
    // ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char *argv[])
{
  mementar::compat::onto_ros::Node::init(argc, argv, "mementarGUI");

  QApplication a(argc, argv);

  a.setStyle(new DarkStyle);

  std::string path = mementar::compat::onto_ros::getShareDirectory("mementar");
  path = path + "/docs/img/logo/mementar.ico";

  // todo: check if the path actually points where it should
  printf("%s\n", path.c_str());

  QIcon icon(QString::fromStdString(path));
  a.setWindowIcon(icon);

  mementarGUI w;
  w.show();

  bool run = true;

  w.init();
  w.wait();

  w.start();

  std::thread spin_thread(spinThread,&run);

  signal(SIGINT, SIG_DFL);
  auto a_exec = a.exec();

  run = false;
  spin_thread.join();

  mementar::compat::onto_ros::Node::shutdown();

  return a_exec;
}
