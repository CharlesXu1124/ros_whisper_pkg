#ifndef WHISPER_CLIENT_CPP__VISIBILITY_CONTROL_H_
#define WHISPER_CLIENT_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WHISPER_CLIENT_CPP_EXPORT __attribute__ ((dllexport))
    #define WHISPER_CLIENT_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define WHISPER_CLIENT_CPP_EXPORT __declspec(dllexport)
    #define WHISPER_CLIENT_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef WHISPER_CLIENT_CPP_BUILDING_DLL
    #define WHISPER_CLIENT_CPP_PUBLIC WHISPER_CLIENT_CPP_EXPORT
  #else
    #define WHISPER_CLIENT_CPP_PUBLIC WHISPER_CLIENT_CPP_IMPORT
  #endif
  #define WHISPER_CLIENT_CPP_PUBLIC_TYPE WHISPER_CLIENT_CPP_PUBLIC
  #define WHISPER_CLIENT_CPP_LOCAL
#else
  #define WHISPER_CLIENT_CPP_EXPORT __attribute__ ((visibility("default")))
  #define WHISPER_CLIENT_CPP_IMPORT
  #if __GNUC__ >= 4
    #define WHISPER_CLIENT_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define WHISPER_CLIENT_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WHISPER_CLIENT_CPP_PUBLIC
    #define WHISPER_CLIENT_CPP_LOCAL
  #endif
  #define WHISPER_CLIENT_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // WHISPER_CLIENT_CPP__VISIBILITY_CONTROL_H_