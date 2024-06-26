#include "mementar/API/mementar/clients/ClientBase.h"

#include <cstddef>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "mementar/compat/ros.h"

namespace mementar {

  std::pair<std::vector<std::string>, compat::mem_ros::Time> ClientBase::call(const std::string& action, const std::string& param)
  {
    cpt++;

    auto req = mementar::compat::makeRequest<mementar::compat::MementarService>();
    auto res = mementar::compat::makeResponse<mementar::compat::MementarService>();

    [action, param](auto&& req) {
      req->action = action;
      req->param = param;
    }(mementar::compat::mem_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_)::RosStatus_e;

    switch(client_.call(req, res))
    {
    case ResultTy::ros_status_successful_with_retries:
    {
      if(client_verbose)
        std::cout << COLOR_GREEN << "Restored mementar/" << name_ << COLOR_OFF << std::endl;
      [[fallthrough]];
    }
    case ResultTy::ros_status_successful:
    {
      return [&](auto&& res) {
        error_code_ = res->code;
        return std::make_pair(res->values, compat::mem_ros::Time(res->time_value.seconds, res->time_value.nanoseconds));
      }(mementar::compat::mem_ros::getServicePointer(res));
    }
    case ResultTy::ros_status_failure:
      [[fallthrough]];
    default:
    {
      error_code_ = -1;

      if(client_verbose)
        std::cout << COLOR_RED << "Failure to callStrs mementar/" << name_ << COLOR_OFF << std::endl;

      return {{}, compat::mem_ros::Time(0)};
    }
    }
  }

  size_t ClientBase::cpt = 0;
  bool ClientBase::client_verbose = false;

} // namespace mementar
