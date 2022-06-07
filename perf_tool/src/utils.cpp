// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "utils.hpp"

#include <iomanip>
#include <sstream>
#include <string>

namespace perf_tool
{

std::string
stringify_gid(const rmw_gid_t & rmw_gid)
{
  std::ostringstream oss;
  oss << std::hex;
  auto it = std::begin(rmw_gid.data);
  const auto last_elem_it = std::end(rmw_gid.data) - 1;
  for (; it != last_elem_it; ++it) {
    // need to cast to integer type, to avoid trying to use utf8 encoding
    oss << std::setw(2) << std::setfill('0') << static_cast<int>(*it) << ".";
  }
  oss << std::setw(2) << std::setfill('0') << static_cast<int>(*last_elem_it);
  return oss.str();
}
}  // namespace perf_tool
