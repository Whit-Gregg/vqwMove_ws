#ifndef VQW_UTIL__VISIBILITY_CONTROL_H_
#define VQW_UTIL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VQW_UTIL_EXPORT __attribute__ ((dllexport))
    #define VQW_UTIL_IMPORT __attribute__ ((dllimport))
  #else
    #define VQW_UTIL_EXPORT __declspec(dllexport)
    #define VQW_UTIL_IMPORT __declspec(dllimport)
  #endif
  #ifdef VQW_UTIL_BUILDING_LIBRARY
    #define VQW_UTIL_PUBLIC VQW_UTIL_EXPORT
  #else
    #define VQW_UTIL_PUBLIC VQW_UTIL_IMPORT
  #endif
  #define VQW_UTIL_PUBLIC_TYPE VQW_UTIL_PUBLIC
  #define VQW_UTIL_LOCAL
#else
  #define VQW_UTIL_EXPORT __attribute__ ((visibility("default")))
  #define VQW_UTIL_IMPORT
  #if __GNUC__ >= 4
    #define VQW_UTIL_PUBLIC __attribute__ ((visibility("default")))
    #define VQW_UTIL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VQW_UTIL_PUBLIC
    #define VQW_UTIL_LOCAL
  #endif
  #define VQW_UTIL_PUBLIC_TYPE
#endif

#endif  // VQW_UTIL__VISIBILITY_CONTROL_H_
