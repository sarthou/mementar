#include "mementar/API/mementar/clients/ClientBase.h"

namespace mementar {
  mementar::compat::onto_ros::ServiceWrapper<mementar::compat::MementarService::Response> ClientBase::call(const std::string& action, const std::string& param)
  {
    cpt++;

    auto req = mementar::compat::make_request<mementar::compat::MementarService>();
    auto res = mementar::compat::make_response<mementar::compat::MementarService>();

    [action, param](auto&& req) {
      req->action = action;
      req->param = param;
    }(mementar::compat::onto_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_)::Status;

    switch(client_.call(req, res))
    {
    case ResultTy::SUCCESSFUL_WITH_RETRIES:
    {
      error_code_ = 0;

      if(verbose_)
      {
        std::cout << COLOR_GREEN << "Restored mementar/" << name_ << COLOR_OFF << std::endl;
      }
      [[fallthrough]];
    }
    case ResultTy::SUCCESSFUL:
    {
      return res;
    }
    case ResultTy::FAILURE:
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

  std::int16_t ClientBase::callCode(const std::string& action, const std::string& param) {
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
    int16_t code;
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
