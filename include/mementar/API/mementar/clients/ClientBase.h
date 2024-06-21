#ifndef MEMENTAR_CLIENTBASE_H
#define MEMENTAR_CLIENTBASE_H

#include <string>
#include <utility>
#include <vector>

#include "mementar/compat/ros.h"

#ifndef COLOR_OFF
#define COLOR_OFF "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED "\x1B[0;91m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE "\x1B[1;33m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN "\x1B[1;92m"
#endif

namespace mementar {
  class ClientBase
  {
  public:
    explicit ClientBase(const std::string& name) : name_(name),
                                                   error_code_(0),
                                                   client_("mementar/" + name) {}

    size_t nb() const { return cpt; }
    void resetNb() { cpt = 0; }
    static void verbose(bool do_verbose) { client_verbose = do_verbose; }
    int getErrorCode() const { return error_code_; }

    std::pair<std::vector<std::string>, compat::mem_ros::Time> call(const std::string& action, const std::string& param);

    std::vector<std::string> callStrs(const std::string& action, const std::string& param)
    {
      return call(action, param).first;
    }

    compat::mem_ros::Time callStamp(const std::string& action, const std::string& param)
    {
      return call(action, param).second;
    }

    /// @brief Calls the service set up in the constructor of ClientBase.
    /// @param action the query action.
    /// @param param the query parameters.
    /// @return Returns a single string. If the service call fails, the returned value is "ERR:SERVICE_FAIL".
    std::string callStr(const std::string& action, const std::string& param)
    {
      auto res = this->callStrs(action, param);
      return res.empty() ? "" : res[0];
    }

    /// @brief Calls the service set up in the constructor of ClientBase.
    /// @param action the query action.
    /// @param param the query parameters.
    /// @return Returns false if the service call fails.
    bool callNR(const std::string& action, const std::string& param)
    {
      return this->callStr(action, param) != "ERR:SERVICE_FAIL";
    }

    /// @brief Calls the service set up in the constructor of ClientBase.
    /// @param action the query action.
    /// @param param the query parameters.
    /// @return Returns false if the service call fails or the result code of the service is different from SUCCESS.
    bool callBool(const std::string& action, const std::string& param)
    {
      auto res = this->callStr(action, param);
      return (res != "ERR:SERVICE_FAIL") && (error_code_ == 0);
    }

  private:
    std::string name_;
    int error_code_ = 0;
    static size_t cpt;
    static bool client_verbose;

  protected:
    compat::mem_ros::Client<compat::MementarService> client_;
  };

} // namespace mementar

#endif // MEMENTAR_CLIENTBASE_H
