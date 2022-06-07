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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <thread>

#include "netperf_tool_interfaces/msg/bytes.hpp"
#include "netperf_tool_interfaces/srv/get_results.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"

#include "client_node.hpp"
#include "node_runner.hpp"
#include "server_node.hpp"
#include "utils.hpp"

// Ignore -Wunused-value for clang.
// Based on https://github.com/pybind/pybind11/issues/2225
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-value"
#endif
#include <pybind11/pybind11.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

using namespace std::chrono_literals;

PYBIND11_MODULE(netperf_tool_impl, m) {
  m.doc() = "Python wrapper of perf tool implementation";
  using ClientNode = netperf_tool::ClientNode;
  using ServerNode = netperf_tool::ServerNode;
  using ClientRunner = netperf_tool::ClientRunner;
  using ServerRunner = netperf_tool::NodeRunner<ServerNode>;
  using ClientCollectedInfo = netperf_tool::ClientCollectedInfo;
  using ServerCollectedInfo = netperf_tool::ServerCollectedInfo;
  using ClientResults = netperf_tool::ClientResults;
  using ServerResults = netperf_tool::ServerResults;
  using Statistics = netperf_tool_interfaces::srv::GetResults::Response;

  pybind11::class_<ClientNode, std::shared_ptr<ClientNode>>(
    m, "ClientNode")
  .def("get_topic_name", &ClientNode::get_topic_name)
  .def("get_stringified_pub_gid", &ClientNode::get_stringified_pub_gid)
  .def("extract_results", &ClientNode::extract_results);
  pybind11::class_<ServerNode, std::shared_ptr<ServerNode>>(
    m, "ServerNode")
  .def("get_topic_name", &ServerNode::get_topic_name)
  .def("extract_new_clients", &ServerNode::extract_new_clients)
  .def("extract_results", &ServerNode::extract_results)
  .def("wait_for_results_available", &ServerNode::wait_for_results_available);
  pybind11::class_<ClientRunner>(
    m, "ClientRunner")
  .def(pybind11::init<rmw_qos_profile_t, size_t, std::chrono::nanoseconds>())
  .def("start", &ClientRunner::start)
  .def("stop", &ClientRunner::stop)
  .def("join", &ClientRunner::join)
  .def("get_node", &ClientRunner::get_node);
  pybind11::class_<ServerRunner>(
    m, "ServerRunner")
  .def(pybind11::init<rmw_qos_profile_t>())
  .def("start", &ServerRunner::start)
  .def("stop", &ServerRunner::stop)
  .def("join", &ServerRunner::join)
  .def("get_node", &ServerRunner::get_node);
  pybind11::class_<ClientCollectedInfo>(
    m, "ClientCollectedInfo")
  .def_readwrite("message_ids", &ClientCollectedInfo::message_ids)
  .def_readwrite(
    "message_published_times",
    &ClientCollectedInfo::message_published_times)
  .def_readwrite("message_sizes", &ClientCollectedInfo::message_sizes);
  pybind11::class_<ServerCollectedInfo>(
    m, "ServerCollectedInfo")
  .def_readwrite("message_ids", &ServerCollectedInfo::message_ids)
  .def_readwrite("message_latencies", &ServerCollectedInfo::message_latencies)
  .def_readwrite("message_sizes", &ServerCollectedInfo::message_sizes);
  pybind11::class_<ClientResults>(
    m, "ClientResults")
  .def_readwrite("collected_info", &ClientResults::collected_info)
  .def_readwrite("statistics", &ClientResults::statistics);
  pybind11::class_<ServerResults>(
    m, "ServerResults")
  .def_readwrite("collected_info", &ServerResults::collected_info)
  .def_readwrite("statistics", &ServerResults::statistics);
  pybind11::class_<netperf_tool_interfaces::srv::GetResults::Response>(
    m, "Statistics")
  .def_readwrite("latency_avg_ms", &Statistics::latency_avg_ms)
  .def_readwrite("latency_stdev_ms", &Statistics::latency_stdev_ms)
  .def_readwrite("latency_min_ms", &Statistics::latency_min_ms)
  .def_readwrite("latency_max_ms", &Statistics::latency_max_ms)
  .def_readwrite("total_bytes", &Statistics::total_bytes)
  .def_readwrite("experiment_duration_ns", &Statistics::experiment_duration_ns)
  .def_readwrite("messages_lost", &Statistics::messages_lost)
  .def_readwrite("messages_total", &Statistics::messages_total);
}
