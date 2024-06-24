#include <QApplication>
#include <array>
#include <csignal>
#include <cstdio>
#include <cstdlib>
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

void spinThread()
{
  mementar::compat::mem_ros::Node::get().spin();
}

int main(int argc, char* argv[])
{
  signal(SIGSEGV, handler);

  mementar::compat::mem_ros::Node::init(argc, argv, "mementarGUI");

  std::thread spin_thread(spinThread);

  QApplication a(argc, argv);

  QApplication::setStyle(new DarkStyle);

  std::string path = mementar::compat::mem_ros::getShareDirectory("mementar");
  path = path + "/docs/img/logo/mementar.ico";

  QIcon icon(QString::fromStdString(path));
  QApplication::setWindowIcon(icon);

  mementarGUI w;
  w.show();

  w.init();
  w.wait();

  w.start();

  signal(SIGINT, SIG_DFL);
  auto a_exec = QApplication::exec();

  mementar::compat::mem_ros::Node::shutdown();
  spin_thread.join();

  return a_exec;
}
