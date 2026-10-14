// Copyright 2026 fruit_ws team
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

/**
 * @file foxglove_service_timeout.cpp
 * @brief Intercepts rclcpp::ClientBase::wait_for_service_nanoseconds to override
 *        foxglove_bridge's hardcoded 1-second timeout to 5 seconds (or FOXGLOVE_SERVICE_TIMEOUT_SEC).
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <errno.h>

extern "C" {

typedef bool (* orig_wait_fn)(void * this_ptr, int64_t nanoseconds);

static int64_t get_target_timeout_ns()
{
  const char * env = getenv("FOXGLOVE_SERVICE_TIMEOUT_SEC");
  if (env != nullptr) {
    char * end = nullptr;
    long sec = strtol(env, &end, 10);
    if (sec > 0 && end != env) {
      return static_cast<int64_t>(sec) * 1000000000LL;
    }
  }
  return 5000000000LL;  // 5 seconds default
}

/**
 * Mangle signature for:
 * rclcpp::ClientBase::wait_for_service_nanoseconds(std::chrono::duration<long, std::ratio<1, 1000000000>>)
 */
bool
_ZN6rclcpp10ClientBase28wait_for_service_nanosecondsENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE(
  void * this_ptr, int64_t nanoseconds)
{
  static orig_wait_fn orig = reinterpret_cast<orig_wait_fn>(dlsym(
      RTLD_NEXT,
      "_ZN6rclcpp10ClientBase28wait_for_service_nanosecondsENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE"));
  if (orig == nullptr) {
    return false;
  }

  // If the caller is foxglove_bridge and specifies 1 second, bump to target timeout (default 5s)
  if (nanoseconds == 1000000000LL) {
    const char * prog = program_invocation_short_name;
    if (prog != nullptr && strstr(prog, "foxglove") != nullptr) {
      nanoseconds = get_target_timeout_ns();
    }
  }

  return orig(this_ptr, nanoseconds);
}

}  // extern "C"
