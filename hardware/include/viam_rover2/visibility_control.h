// Copyright 2021 ros2_control Development Team
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
//
// This file was last modified by Michael Hart, a.k.a Mike Likes Robots
// (mikelikesrobots@outlook.com), on 2024-03-07.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef VIAM_ROVER2__VISIBILITY_CONTROL_H_
#define VIAM_ROVER2__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define VIAM_ROVER2_EXPORT __attribute__((dllexport))
#define VIAM_ROVER2_IMPORT __attribute__((dllimport))
#else
#define VIAM_ROVER2_EXPORT __declspec(dllexport)
#define VIAM_ROVER2_IMPORT __declspec(dllimport)
#endif
#ifdef VIAM_ROVER2_BUILDING_DLL
#define VIAM_ROVER2_PUBLIC VIAM_ROVER2_EXPORT
#else
#define VIAM_ROVER2_PUBLIC VIAM_ROVER2_IMPORT
#endif
#define VIAM_ROVER2_PUBLIC_TYPE VIAM_ROVER2_PUBLIC
#define VIAM_ROVER2_LOCAL
#else
#define VIAM_ROVER2_EXPORT __attribute__((visibility("default")))
#define VIAM_ROVER2_IMPORT
#if __GNUC__ >= 4
#define VIAM_ROVER2_PUBLIC __attribute__((visibility("default")))
#define VIAM_ROVER2_LOCAL __attribute__((visibility("hidden")))
#else
#define VIAM_ROVER2_PUBLIC
#define VIAM_ROVER2_LOCAL
#endif
#define VIAM_ROVER2_PUBLIC_TYPE
#endif

#endif  // VIAM_ROVER2__VISIBILITY_CONTROL_H_
