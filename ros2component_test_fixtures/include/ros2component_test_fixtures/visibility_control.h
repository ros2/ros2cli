// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2COMPONENT_TEST_FIXTURES__VISIBILITY_CONTROL_H_
#define ROS2COMPONENT_TEST_FIXTURES__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROS2COMPONENT_TEST_FIXTURES_EXPORT __attribute__ ((dllexport))
    #define ROS2COMPONENT_TEST_FIXTURES_IMPORT __attribute__ ((dllimport))
  #else
    #define ROS2COMPONENT_TEST_FIXTURES_EXPORT __declspec(dllexport)
    #define ROS2COMPONENT_TEST_FIXTURES_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROS2COMPONENT_TEST_FIXTURES_BUILDING_DLL
    #define ROS2COMPONENT_TEST_FIXTURES_PUBLIC ROS2COMPONENT_TEST_FIXTURES_EXPORT
  #else
    #define ROS2COMPONENT_TEST_FIXTURES_PUBLIC ROS2COMPONENT_TEST_FIXTURES_IMPORT
  #endif
  #define ROS2COMPONENT_TEST_FIXTURES_PUBLIC_TYPE ROS2COMPONENT_TEST_FIXTURES_PUBLIC
  #define ROS2COMPONENT_TEST_FIXTURES_LOCAL
#else
  #define ROS2COMPONENT_TEST_FIXTURES_EXPORT __attribute__ ((visibility("default")))
  #define ROS2COMPONENT_TEST_FIXTURES_IMPORT
  #if __GNUC__ >= 4
    #define ROS2COMPONENT_TEST_FIXTURES_PUBLIC __attribute__ ((visibility("default")))
    #define ROS2COMPONENT_TEST_FIXTURES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROS2COMPONENT_TEST_FIXTURES_PUBLIC
    #define ROS2COMPONENT_TEST_FIXTURES_LOCAL
  #endif
  #define ROS2COMPONENT_TEST_FIXTURES_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROS2COMPONENT_TEST_FIXTURES__VISIBILITY_CONTROL_H_
