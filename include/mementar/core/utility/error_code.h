#ifndef MEMENTAR_ERRORCODE_H
#define MEMENTAR_ERRORCODE_H

#define NO_ERROR          (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::NoEffect)
#define REQUEST_ERROR     (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::RequestError)
#define UNKNOW_ACTION     (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::UnknownAction)
#define UNINIT            (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::NotInitialized)
#define NO_EFFECT         (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::NoEffect)
#define OTHER             (mementar::reserved::triggerErrMacroDeprecated(), mementar::ServiceCode::Unknown)

namespace mementar {
    namespace reserved {
        [[deprecated("The use of macros is deprecated, please refer to mementar::ServiceCode from now on")]]
        inline void triggerErrMacroDeprecated() {}
    }

    struct ServiceCode {
        enum Enum : int16_t {
            NoError = 0,
            RequestError = 1,
            UnknownAction = 2,
            NotInitialized = 3,
            NoEffect = 4,
            Unknown = -1
        };
    };
}

#endif // MEMENTAR_ERRORCODE_H