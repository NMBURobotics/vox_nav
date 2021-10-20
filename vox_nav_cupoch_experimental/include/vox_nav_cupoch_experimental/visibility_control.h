// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef CUPOCH_CONVERSIONS__VISIBILITY_CONTROL_H_
#define CUPOCH_CONVERSIONS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CUPOCH_CONVERSIONS_EXPORT __attribute__ ((dllexport))
    #define CUPOCH_CONVERSIONS_IMPORT __attribute__ ((dllimport))
  #else
    #define CUPOCH_CONVERSIONS_EXPORT __declspec(dllexport)
    #define CUPOCH_CONVERSIONS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CUPOCH_CONVERSIONS_BUILDING_LIBRARY
    #define CUPOCH_CONVERSIONS_PUBLIC CUPOCH_CONVERSIONS_EXPORT
  #else
    #define CUPOCH_CONVERSIONS_PUBLIC CUPOCH_CONVERSIONS_IMPORT
  #endif
  #define CUPOCH_CONVERSIONS_PUBLIC_TYPE CUPOCH_CONVERSIONS_PUBLIC
  #define CUPOCH_CONVERSIONS_LOCAL
#else
  #define CUPOCH_CONVERSIONS_EXPORT __attribute__ ((visibility("default")))
  #define CUPOCH_CONVERSIONS_IMPORT
  #if __GNUC__ >= 4
    #define CUPOCH_CONVERSIONS_PUBLIC __attribute__ ((visibility("default")))
    #define CUPOCH_CONVERSIONS_LOCAL  __attribute__ ((visibility("default")))
  #else
    #define CUPOCH_CONVERSIONS_PUBLIC
    #define CUPOCH_CONVERSIONS_LOCAL
  #endif
  #define CUPOCH_CONVERSIONS_PUBLIC_TYPE
#endif

#endif  // CUPOCH_CONVERSIONS__VISIBILITY_CONTROL_H_
