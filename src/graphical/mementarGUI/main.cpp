#include <QApplication>
#include <array>
#include <csignal>
#include <cstdio>
#include <execinfo.h>
#include <string>
#include <thread>
#include <unistd.h>

#include "include/mementar/graphical/mementarGUI/DarkStyle.h"
#include "include/mementar/graphical/mementarGUI/mementargui.h"
#include "mementar/compat/ros.h"
#include "qapplication.h"
#include "qicon.h"
#include "qstring.h"

void handler(int sig)
{
  std::array<void*, 10> array;
  int size = backtrace(array.data(), 10);

  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array.data(), size, STDERR_FILENO);
  exit(1);
}

int main(int argc, char* argv[])
{
  signal(SIGSEGV, handler);

  mementar::compat::onto_ros::Node::init(argc, argv, "mementarGUI");

  std::thread th([]() {
    mementar::compat::onto_ros::Node::get().spin();

    while(mementar::compat::onto_ros::Node::ok())
    {
      usleep(5000);
    }
  });

  QApplication a(argc, argv);

  QApplication::setStyle(new DarkStyle);

  std::string path = mementar::compat::onto_ros::getShareDirectory("mementar");
  path = path + "/docs/img/logo/mementar.ico";

  QIcon icon(QString::fromStdString(path));
  QApplication::setWindowIcon(icon);

  mementarGUI w;
  w.show();

  bool run = true;

  w.init();
  w.wait();

  w.start();

  signal(SIGINT, SIG_DFL);
  auto a_exec = QApplication::exec();

  run = false;

  mementar::compat::onto_ros::Node::shutdown();
  th.join();

  return a_exec;
}
