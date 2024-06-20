#include <algorithm>
#include <array>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <execinfo.h>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "mementar/RosInterface.h"
#include "mementar/compat/ros.h"
#include "mementar/core/Parametrization/Parameters.h"
#include "mementar/core/utility/error_code.h"

void handler(int sig)
{
  std::array<void*, 10> array;
  int size = 0;

  size = backtrace(array.data(), 10);

  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array.data(), size, STDERR_FILENO);
  exit(1);
}

void removeUselessSpace(std::string& text)
{
  while((text[0] == ' ') && (text.empty() == false))
  {
    text.erase(0, 1);
  }

  while((text[text.size() - 1] == ' ') && (text.empty() == false))
  {
    text.erase(text.size() - 1, 1);
  }
}

std::map<std::string, std::unique_ptr<mementar::RosInterface>> interfaces;
std::map<std::string, std::thread> interfaces_threads;

mementar::Parameters params;

bool deleteInterface(const std::string& name)
{
  interfaces[name]->stop();
  usleep(1000);

  try
  {
    interfaces_threads[name].join();
  }
  catch(std::runtime_error& ex)
  {
    mementar::Display::error("Catch error when joining the interface thread : " + std::string(ex.what()));
    mementar::Display::warning("The thread will be detached");
    interfaces_threads[name].detach();
  }

  interfaces_threads.erase(name);
  interfaces.erase(name);

  std::cout << name << " STOPPED" << std::endl;
  return true;
}

bool managerHandle(mementar::compat::onto_ros::ServiceWrapper<mementar::compat::MementarService::Request>& req,
                   mementar::compat::onto_ros::ServiceWrapper<mementar::compat::MementarService::Response>& res)
{
  return [](auto&& req, auto&& res) {
    res->code = mementar::ServiceCode::service_no_error;

    removeUselessSpace(req->action);
    removeUselessSpace(req->param);

    if(req->action == "add")
    {
      auto it = interfaces.find(req->param);
      if(it != interfaces.end())
      {
        res->code = mementar::ServiceCode::service_no_effect;
      }
      else
      {
        auto tmp = std::make_unique<mementar::RosInterface>(params.parameters_.at("directory").getFirst(),
                                                            params.parameters_.at("config").getFirst(),
                                                            10,
                                                            req->param);

        std::thread th(&mementar::RosInterface::run, tmp.get());

        interfaces[req->param] = std::move(tmp);
        interfaces_threads[req->param] = std::move(th);

        std::cout << req->param << " STARTED" << std::endl;
      }
    }
    else if(req->action == "delete")
    {
      auto it = interfaces.find(req->param);

      if(it == interfaces.end())
      {
        res->code = mementar::ServiceCode::service_no_effect;
      }
      else
      {
        if(deleteInterface(req->param) == false)
          res->code = mementar::ServiceCode::service_request_error;
      }
    }
    else if(req->action == "list")
    {
      for(const auto& [name, _] : interfaces)
      {
        res->values.push_back(name);
      }
    }
    else
    {
      res->code = mementar::ServiceCode::service_unknown_action;
    }

    return true;
  }(mementar::compat::onto_ros::getServicePointer(req), mementar::compat::onto_ros::getServicePointer(res));
}

int main(int argc, char** argv)
{
  signal(SIGSEGV, handler);
  mementar::compat::onto_ros::Node::init(argc, argv, "mementar_multi");

  params.insert(mementar::Parameter("directory", {"-d", "--directory"}, {"none"}));
  params.insert(mementar::Parameter("config", {"-c", "--config"}, {"none"}));

  params.set(argc, argv);
  params.display();

  mementar::compat::onto_ros::Service<mementar::compat::MementarService> service("/mementar/manage", managerHandle);
  mementar::compat::onto_ros::Node::get().spin();

  while(mementar::compat::onto_ros::Node::ok())
  {
    usleep(1);
  }

  std::vector<std::string> interfaces_names;
  std::transform(interfaces.cbegin(),
                 interfaces.cend(),
                 std::back_inserter(interfaces_names),
                 [](const auto& interface) { return interface.first; });

  for(const auto& interfaces_names : interfaces_names)
  {
    deleteInterface(interfaces_names);
  }

  mementar::compat::onto_ros::Node::shutdown();

  // ROS_DEBUG("KILL mementar");

  return 0;
}
