#include "include/mementar/graphical/mementarGUI/DarkStyle.h"
#include "include/mementar/graphical/mementarGUI/mementargui.h"

#include <QApplication>

#include <csignal>
#include <thread>

#include "mementar/compat/ros.h"

#include <execinfo.h>

void handler(int sig) {
    void *array[10];
    size_t size;

    size = backtrace(array, 10);

    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

int main(int argc, char *argv[]) {
    signal(SIGSEGV, handler);

    mementar::compat::onto_ros::Node::init(argc, argv, "mementarGUI");

    std::thread th([]() {
        while (mementar::compat::onto_ros::Node::ok()) {
            mementar::compat::onto_ros::Node::get().spin();
            usleep(5000);
        }
    });

    QApplication a(argc, argv);

    a.setStyle(new DarkStyle);

    std::string path = mementar::compat::onto_ros::getShareDirectory("mementar");
    path = path + "/docs/img/logo/mementar.ico";

    QIcon icon(QString::fromStdString(path));
    a.setWindowIcon(icon);

    mementarGUI w;
    w.show();

    bool run = true;

    w.init();
    w.wait();

    w.start();

    signal(SIGINT, SIG_DFL);
    auto a_exec = a.exec();

    run = false;

    mementar::compat::onto_ros::Node::shutdown();
    th.join();

    return a_exec;
}
