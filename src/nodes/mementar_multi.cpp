#include <thread>
#include <execinfo.h>

#include <mementar/compat/ros.h>

#include "mementar/RosInterface.h"
#include "mementar/core/utility/error_code.h"
#include "mementar/core/Parametrization/Parameters.h"

void handler(int sig) {
    void *array[10];
    size_t size;

    size = backtrace(array, 10);

    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

void removeUselessSpace(std::string& text) {
    while ((text[0] == ' ') && (text.size() != 0)) {
        text.erase(0, 1);
    }

    while ((text[text.size() - 1] == ' ') && (text.size() != 0)) {
        text.erase(text.size() - 1, 1);
    }
}

std::map<std::string, std::unique_ptr<mementar::RosInterface> > interfaces_;
std::map<std::string, std::thread> interfaces_threads_;

mementar::Parameters params;

bool deleteInterface(std::string name) {
    interfaces_[name]->stop();
    usleep(1000);

    try {
        interfaces_threads_[name].join();
    } catch (std::runtime_error& ex) {
        mementar::Display::error("Catch error when joining the interface thread : " + std::string(ex.what()));
        mementar::Display::warning("The thread will be detached");
        interfaces_threads_[name].detach();
    }

    interfaces_threads_.erase(name);
    interfaces_.erase(name);

    std::cout << name << " STOPPED" << std::endl;
    return true;
}

bool managerHandle(mementar::compat::onto_ros::ServiceWrapper<mementar::compat::MementarService::Request>& req,
                   mementar::compat::onto_ros::ServiceWrapper<mementar::compat::MementarService::Response>& res) {
    return [](auto&& req, auto&& res) {
        res->code = mementar::ServiceCode::NoError;

        removeUselessSpace(req->action);
        removeUselessSpace(req->param);

        if (req->action == "add") {
            auto it = interfaces_.find(req->param);
            if (it != interfaces_.end())
                res->code = mementar::ServiceCode::NoEffect;
            else {
                auto tmp = std::make_unique<mementar::RosInterface>(params.parameters_.at("directory").getFirst(),
                                                                    params.parameters_.at("config").getFirst(),
                                                                    10,
                                                                    req->param);

                std::thread th(&mementar::RosInterface::run, tmp.get());

                interfaces_[req->param] = std::move(tmp);
                interfaces_threads_[req->param] = std::move(th);

                std::cout << req->param << " STARTED" << std::endl;
            }
        } else if (req->action == "delete") {
            auto it = interfaces_.find(req->param);

            if (it == interfaces_.end()) {
                res->code = mementar::ServiceCode::NoEffect;
            } else {
                if (deleteInterface(req->param) == false)
                    res->code = mementar::ServiceCode::RequestError;
            }
        } else if (req->action == "list") {
            for (const auto& [name, _] : interfaces_) {
                res->values.push_back(name);
            }
        } else {
            res->code = mementar::ServiceCode::UnknownAction;
        }

        return true;
    }(mementar::compat::onto_ros::getServicePointer(req), mementar::compat::onto_ros::getServicePointer(res));
}

int main(int argc, char** argv) {
    signal(SIGSEGV, handler);
    mementar::compat::onto_ros::Node::init(argc, argv, "mementar_multi");

    params.insert(mementar::Parameter("directory", {"-d", "--directory"}, {"none"}));
    params.insert(mementar::Parameter("config", {"-c", "--config"}, {"none"}));

    params.set(argc, argv);
    params.display();

    mementar::compat::onto_ros::Service<mementar::compat::MementarService> service("/mementar/manage", managerHandle);
    mementar::compat::onto_ros::Node::get().spin();

    std::vector<std::string> interfaces_names;
    std::transform(interfaces_.cbegin(),
                   interfaces_.cend(),
                   std::back_inserter(interfaces_names),
                   [](const auto& interface) { return interface.first; });

    for (size_t i = 0; i < interfaces_names.size(); i++) {
        deleteInterface(interfaces_names[i]);
    }

    while (mementar::compat::onto_ros::Node::ok()) usleep(1);

    mementar::compat::onto_ros::Node::shutdown();

    // ROS_DEBUG("KILL mementar");

    return 0;
}
