#include "mementar/core/Parametrization/Parameters.h"
#include "mementar/RosInterface.h"

#include <execinfo.h>

void handler(int sig) {
    void *array[10];
    size_t size;

    size = backtrace(array, 10);

    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

int main(int argc, char** argv) {
    signal(SIGSEGV, handler);
    mementar::compat::onto_ros::Node::init(argc, argv, "mementar_single");

    std::thread th([]() { mementar::compat::onto_ros::Node::get().spin(); });

    mementar::Parameters params;
    params.insert(mementar::Parameter("directory", {"-d", "--directory"}, {"none"}));
    params.insert(mementar::Parameter("config", {"-c", "--config"}, {"none"}));

    params.set(argc, argv);
    params.display();

    mementar::RosInterface interface(
        params.parameters_.at("directory").getFirst(),
        params.parameters_.at("config").getFirst()
    );
    interface.run();

    mementar::compat::onto_ros::Node::shutdown();
    th.join();

    return 0;
}
