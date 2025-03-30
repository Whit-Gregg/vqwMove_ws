#ifndef JOY_PAN_TILT__VISIBILITY_CONTROL_H_
#define JOY_PAN_TILT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JOY_PAN_TILT_EXPORT __attribute__ ((dllexport))
    #define JOY_PAN_TILT_IMPORT __attribute__ ((dllimport))
  #else
    #define JOY_PAN_TILT_EXPORT __declspec(dllexport)
    #define JOY_PAN_TILT_IMPORT __declspec(dllimport)
  #endif
  #ifdef JOY_PAN_TILT_BUILDING_LIBRARY
    #define JOY_PAN_TILT_PUBLIC JOY_PAN_TILT_EXPORT
  #else
    #define JOY_PAN_TILT_PUBLIC JOY_PAN_TILT_IMPORT
  #endif
  #define JOY_PAN_TILT_PUBLIC_TYPE JOY_PAN_TILT_PUBLIC
  #define JOY_PAN_TILT_LOCAL
#else
  #define JOY_PAN_TILT_EXPORT __attribute__ ((visibility("default")))
  #define JOY_PAN_TILT_IMPORT
  #if __GNUC__ >= 4
    #define JOY_PAN_TILT_PUBLIC __attribute__ ((visibility("default")))
    #define JOY_PAN_TILT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JOY_PAN_TILT_PUBLIC
    #define JOY_PAN_TILT_LOCAL
  #endif
  #define JOY_PAN_TILT_PUBLIC_TYPE
#endif

#endif  // JOY_PAN_TILT__VISIBILITY_CONTROL_H_
