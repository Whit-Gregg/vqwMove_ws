#ifndef CAM_PAN_TILT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define CAM_PAN_TILT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CAM_PAN_TILT_HARDWARE_INTERFACE_BUILDING_LIBRARY
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_PUBLIC CAM_PAN_TILT_HARDWARE_INTERFACE_EXPORT
  #else
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_PUBLIC CAM_PAN_TILT_HARDWARE_INTERFACE_IMPORT
  #endif
  #define CAM_PAN_TILT_HARDWARE_INTERFACE_PUBLIC_TYPE CAM_PAN_TILT_HARDWARE_INTERFACE_PUBLIC
  #define CAM_PAN_TILT_HARDWARE_INTERFACE_LOCAL
#else
  #define CAM_PAN_TILT_HARDWARE_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define CAM_PAN_TILT_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_PUBLIC
    #define CAM_PAN_TILT_HARDWARE_INTERFACE_LOCAL
  #endif
  #define CAM_PAN_TILT_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // CAM_PAN_TILT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
