#ifndef BNO08X_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define BNO08X_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BNO08X_HARDWARE_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define BNO08X_HARDWARE_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define BNO08X_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define BNO08X_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef BNO08X_HARDWARE_INTERFACE_BUILDING_LIBRARY
    #define BNO08X_HARDWARE_INTERFACE_PUBLIC BNO08X_HARDWARE_INTERFACE_EXPORT
  #else
    #define BNO08X_HARDWARE_INTERFACE_PUBLIC BNO08X_HARDWARE_INTERFACE_IMPORT
  #endif
  #define BNO08X_HARDWARE_INTERFACE_PUBLIC_TYPE BNO08X_HARDWARE_INTERFACE_PUBLIC
  #define BNO08X_HARDWARE_INTERFACE_LOCAL
#else
  #define BNO08X_HARDWARE_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define BNO08X_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define BNO08X_HARDWARE_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define BNO08X_HARDWARE_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BNO08X_HARDWARE_INTERFACE_PUBLIC
    #define BNO08X_HARDWARE_INTERFACE_LOCAL
  #endif
  #define BNO08X_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // BNO08X_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
