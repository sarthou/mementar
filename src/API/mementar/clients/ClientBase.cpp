#include "mementar/API/mementar/clients/ClientBase.h"

namespace mementar {
  mementar::compat::onto_ros::ServiceWrapper<mementar::compat::MementarService::Response> ClientBase::call(const std::string& action, const std::string& param, int16_t& code)
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
      return [&](auto&& res) {
        code = res->code;
        return res;
      }(mementar::compat::onto_ros::getServicePointer(res));
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

  std::vector<std::string> ClientBase::callArray(const std::string& action, const std::string& param, int16_t& code)
  {
    auto res = call(action, param, code);

    if(error_code_ == -1)
    {
      return {"ERR:SERVICE_FAIL"};
    }

    return res->values;
  }

  std::string ClientBase::callStr(const std::string& action, const std::string& param, int16_t& code)
  {
    auto res = this->callArray(action, param, code);
    return res.empty() ? "" : res[0];
  }

  bool ClientBase::callBool(const std::string& action, const std::string& param)
  {
    int16_t code;
    auto res = this->callStr(action, param, code);

    return (res != "ERR:SERVICE_FAIL") && (code == 0);
  }

  bool ClientBase::callNR(const std::string& action, const std::string& param)
  {
    return this->callStr(action, param) != "ERR:SERVICE_FAIL";
  }

  compat::onto_ros::Time ClientBase::callStamp(const std::string& action, const std::string& param, int16_t& code)
  {
    auto res = call(action, param, code)->time_value;

    if(error_code_ == -1)
    {
      return compat::onto_ros::Time(0);
    }

    return compat::onto_ros::Time(res.seconds, res.nanoseconds);
  }

  size_t ClientBase::cpt = 0;
  bool ClientBase::verbose_ = false;
  int16_t ClientBase::ignore_ = 0;

} // namespace mementar
