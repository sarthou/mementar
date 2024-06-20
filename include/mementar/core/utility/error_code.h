#ifndef MEMENTAR_ERRORCODE_H
#define MEMENTAR_ERRORCODE_H

#include <cstdint>

#define NO_ERROR (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::service_no_effect)
#define REQUEST_ERROR (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::service_request_error)
#define UNKNOW_ACTION (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::service_unknown_action)
#define UNINIT (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::service_not_initialized)
#define NO_EFFECT (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::service_no_effect)
#define OTHER (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::service_other)

namespace mementar {
  namespace reserved {
    [[deprecated("The use of macros is deprecated, please refer to mementar::ServiceCode from now on")]]
    inline void triggerErrMacroDeprecated()
    {}
  } // namespace reserved

  struct ServiceCode
  {
    enum Enum : int16_t
    {
      service_no_error = 0,
      service_request_error = 1,
      service_unknown_action = 2,
      service_not_initialized = 3,
      service_no_effect = 4,
      service_other = -1
    };
  };
} // namespace mementar

#endif // MEMENTAR_ERRORCODE_H