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

#include "client_runner.hpp"

#include <chrono>
#include <stdexcept>

namespace netperf_tool
{
void
ClientRunner::start(std::chrono::nanoseconds duration)
{
  if (node_->wait_for_server()) {
    node_->start_publishing();
    return NodeRunner<ClientNode>::start(duration);
  }
  throw std::runtime_error{"failed to connect to server"};
}

void
ClientRunner::join()
{
  bool sync_needed = running_thread_.joinable();
  NodeRunner<ClientNode>::join();
  if (sync_needed) {
    node_->stop_publishing();
    node_->sync_with_server(exec_);
  }
}
}  // namespace netperf_tool
