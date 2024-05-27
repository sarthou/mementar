#ifndef MEMENTAR_CLIENTBASE_H
#define MEMENTAR_CLIENTBASE_H

#include <string>
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
    ClientBase(std::string name) : client_("mementar/" + name), name_(name) {}

    size_t nb() { return cpt; }
    void resetNb() { cpt = 0; }
    static void verbose(bool verbose) { verbose_ = verbose; }
    int getErrorCode() { return error_code_; }

    // todo: get rid of the extra parameter `code` since we can get the error code via `getErrorCode`
    mementar::compat::onto_ros::ServiceWrapper<mementar::compat::MementarService::Response> call(const std::string& action, const std::string& param, int16_t& code = ignore_);
    std::vector<std::string> callArray(const std::string& action, const std::string& param, int16_t& code = ignore_);
    std::string callStr(const std::string& action, const std::string& param, int16_t& code = ignore_);
    bool callBool(const std::string& action, const std::string& param);
    bool callNR(const std::string& action, const std::string& param);
    compat::onto_ros::Time callStamp(const std::string& action, const std::string& param, int16_t& code = ignore_);

  private:
    compat::onto_ros::Client<compat::MementarService> client_;
    std::string name_;
    int error_code_ = 0;
    static size_t cpt;
    static bool verbose_;
    static int16_t ignore_;
  };

} // namespace mementar

#endif // MEMENTAR_CLIENTBASE_H
