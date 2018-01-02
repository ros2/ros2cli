#ifndef @(package_name)__VISIBILITY_CONTROL_H_
#define @(package_name)__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define @(package_name)_EXPORT __attribute__ ((dllexport))
    #define @(package_name)_IMPORT __attribute__ ((dllimport))
  #else
    #define @(package_name)_EXPORT __declspec(dllexport)
    #define @(package_name)_IMPORT __declspec(dllimport)
  #endif
  #ifdef @(package_name)_BUILDING_LIBRARY
    #define @(package_name)_PUBLIC @(package_name)_EXPORT
  #else
    #define @(package_name)_PUBLIC @(package_name)_IMPORT
  #endif
  #define @(package_name)_PUBLIC_TYPE @(package_name)_PUBLIC
  #define @(package_name)_LOCAL
#else
  #define @(package_name)_EXPORT __attribute__ ((visibility("default")))
  #define @(package_name)_IMPORT
  #if __GNUC__ >= 4
    #define @(package_name)_PUBLIC __attribute__ ((visibility("default")))
    #define @(package_name)_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define @(package_name)_PUBLIC
    #define @(package_name)_LOCAL
  #endif
  #define @(package_name)_PUBLIC_TYPE
#endif

#endif  // @(package_name)__VISIBILITY_CONTROL_H_
