#include "mementar/API/mementar/clients/ClientBase.h"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "mementar/compat/ros.h"

namespace mementar {

  mementar::compat::onto_ros::ServiceWrapper<mementar::compat::MementarService::Response> ClientBase::call(const std::string& action, const std::string& param)
  {
    cpt++;

    auto req = mementar::compat::makeRequest<mementar::compat::MementarService>();
    auto res = mementar::compat::makeResponse<mementar::compat::MementarService>();

    [action, param](auto&& req) {
      req->action = action;
      req->param = param;
    }(mementar::compat::onto_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_)::RosStatus_e;

    switch(client_.call(req, res))
    {
    case ResultTy::ros_status_successful_with_retries:
    {
      error_code_ = 0;

      if(verbose_)
      {
        std::cout << COLOR_GREEN << "Restored mementar/" << name_ << COLOR_OFF << std::endl;
      }
      [[fallthrough]];
    }
    case ResultTy::ros_status_successful:
    {
      return res;
    }
    case ResultTy::ros_status_failure:
      [[fallthrough]];
    default:
    {
      error_code_ = -1;

      if(verbose_)
      {
        std::cout << COLOR_RED << "Failure to callArray mementar/" << name_ << COLOR_OFF << std::endl;
      }

      return res;
    }
    }
  }

  std::int16_t ClientBase::callCode(const std::string& action, const std::string& param)
  {
    auto res = call(action, param);
    return compat::onto_ros::getServicePointer(res)->code;
  }

  std::vector<std::string> ClientBase::callArray(const std::string& action, const std::string& param)
  {
    auto res = call(action, param);

    if(error_code_ == -1)
    {
      return {"ERR:SERVICE_FAIL"};
    }

    return compat::onto_ros::getServicePointer(res)->values;
  }

  std::string ClientBase::callStr(const std::string& action, const std::string& param)
  {
    auto res = this->callArray(action, param);
    return res.empty() ? "" : res[0];
  }

  bool ClientBase::callBool(const std::string& action, const std::string& param)
  {
    int16_t code = 0;
    auto res = this->callStr(action, param);

    return (res != "ERR:SERVICE_FAIL") && (code == 0);
  }

  bool ClientBase::callNR(const std::string& action, const std::string& param)
  {
    return this->callStr(action, param) != "ERR:SERVICE_FAIL";
  }

  compat::onto_ros::Time ClientBase::callStamp(const std::string& action, const std::string& param)
  {
    auto res = call(action, param);
    auto res_time = compat::onto_ros::getServicePointer(res)->time_value;

    if(error_code_ == -1)
    {
      return compat::onto_ros::Time(0);
    }

    return compat::onto_ros::Time(res_time.seconds, res_time.nanoseconds);
  }

  size_t ClientBase::cpt = 0;
  bool ClientBase::verbose_ = false;

} // namespace mementar
