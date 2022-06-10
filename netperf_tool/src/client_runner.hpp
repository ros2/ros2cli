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

#ifndef CLIENT_RUNNER_HPP_
#define CLIENT_RUNNER_HPP_

#include <chrono>

#include "client_node.hpp"
#include "node_runner.hpp"

namespace netperf_tool
{

/// Extension of NodeRunner for ClientNode.
class ClientRunner : public NodeRunner<ClientNode>
{
public:
  using NodeRunner<ClientNode>::NodeRunner;
  void start(std::chrono::nanoseconds duration);
  void join();
};
}  // namespace netperf_tool

#endif  // CLIENT_RUNNER_HPP_
