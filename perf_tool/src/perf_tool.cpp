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

#include "perf_tool_msgs/msg/bytes.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"

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

namespace perf_tool
{

struct ServerResults
{
  std::vector<size_t> message_ids;
  std::vector<std::chrono::nanoseconds> message_latencies;
  std::vector<size_t> message_sizes;
};

class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(
    const rclcpp::QoS & sub_qos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : Node("perf_tool_server", options)
  {
    sub_ = this->create_subscription<perf_tool_msgs::msg::Bytes>(
      "perf_topic", sub_qos,
      std::bind(&ServerNode::handle_msg, this, std::placeholders::_1, std::placeholders::_2));
  }

  void handle_msg(
    const rclcpp::SerializedMessage & serialized_msg,
    const rclcpp::MessageInfo & msg_info)
  {
    auto now = std::chrono::system_clock::now();
    std::chrono::nanoseconds latency;
    size_t received_bytes = serialized_msg.size();
    perf_tool_msgs::msg::Bytes msg;
    deserializer_.deserialize_message(&serialized_msg, &msg);
    const auto & rmw_msg_info = msg_info.get_rmw_message_info();
    if (rmw_msg_info.source_timestamp != 0 && rmw_msg_info.received_timestamp != 0) {
      // source and received timestamp supported
      // TODO(ivanpauno): Add a way to evaluate this in rmw_feature_supported() to avoid the if().
      RCLCPP_INFO_ONCE(
        this->get_logger(),
        "source and received timestamp supported, using them to calculate latency!");
      auto pub_timestamp = std::chrono::time_point<std::chrono::system_clock>{
        std::chrono::nanoseconds{rmw_msg_info.source_timestamp}};
      auto rec_timestamp = 
        std::chrono::time_point<std::chrono::system_clock>{
          std::chrono::nanoseconds{rmw_msg_info.received_timestamp}};
      latency = rec_timestamp - pub_timestamp;
    } else {
      // TODO(ivanpauno): Maybe it's worth to always calculate this latency (?)
      // as it takes in account the executor overhead.
      RCLCPP_INFO_ONCE(
        this->get_logger(),
        "either source or received timestamp are not supported,"
        "using timestamps added to the message to calculate latency");
      auto pub_timestamp = 
        std::chrono::time_point<std::chrono::system_clock>{
          std::chrono::nanoseconds{msg.timestamp}};
      latency = now - pub_timestamp;
    }
    {
      std::lock_guard guard{results_mutex_};
      results_.message_ids.emplace_back(msg.id);
      results_.message_latencies.emplace_back(latency);
      results_.message_sizes.emplace_back(received_bytes);
    }
  }

  ServerResults
  get_results() const
  {
    std::lock_guard guard{results_mutex_};
    return results_;
  }

  std::string
  get_topic_name()
  {
    return sub_->get_topic_name();
  }
private:
  ServerResults results_;
  mutable std::mutex results_mutex_;
  rclcpp::Subscription<perf_tool_msgs::msg::Bytes>::SharedPtr sub_;
  rclcpp::Serialization<perf_tool_msgs::msg::Bytes> deserializer_;
};

struct ClientResults
{
  std::vector<size_t> message_ids;
  std::vector<std::chrono::time_point<std::chrono::system_clock>> message_published_times;
  std::vector<size_t> message_sizes;
};

class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(
    size_t array_size,
    std::chrono::nanoseconds target_pub_period,
    const rclcpp::QoS & pub_qos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : Node("perf_tool_client", options), array_size_{array_size}
  {
    pub_ = this->create_publisher<perf_tool_msgs::msg::Bytes>("perf_topic", pub_qos);
    timer_ = this->create_wall_timer(
      target_pub_period,
      std::bind(&ClientNode::pub_next_msg, this));
    // always keep the vector preallocated, to avoid delays when the timer is triggered
    bytes_.resize(array_size);
  }

  void pub_next_msg()
  {
    perf_tool_msgs::msg::Bytes msg;
    rclcpp::SerializedMessage serialized_msg;
    msg.id = next_id;
    auto now = std::chrono::system_clock::now();
    msg.timestamp = now.time_since_epoch().count();
    msg.data = std::move(bytes_);
    serializer_.serialize_message(&msg, &serialized_msg);
    auto sent_bytes = serialized_msg.size();
    pub_->publish(msg);
    // always keep the vector preallocated, to avoid delays when the timer is triggered
    bytes_.resize(array_size_);
    {
      std::lock_guard guard{results_mutex_};
      results_.message_ids.emplace_back(msg.id);
      results_.message_published_times.emplace_back(now);
      results_.message_sizes.emplace_back(sent_bytes);
    }
    ++next_id;
  }

  ClientResults
  get_results()
  {
    std::lock_guard guard{results_mutex_};
    return results_;
  }

  std::string
  get_topic_name()
  {
    return pub_->get_topic_name();
  }

  std::string
  get_stringified_pub_gid()
  {
    std::ostringstream oss;
    oss << std::hex;
    auto it = std::begin(pub_->get_gid().data);
    const auto last_elem_it = std::end(pub_->get_gid().data) - 1;
    for(; it != last_elem_it; ++it) {
      // need to cast to integer type, to avoid trying to use utf8 encoding
      oss << std::setw(2) << std::setfill('0') << static_cast<int>(*it) << ".";
    }
    oss << std::setw(2) << std::setfill('0') << static_cast<int>(*last_elem_it);
    return oss.str();
  }
private:
  size_t array_size_;
  std::vector<unsigned char> bytes_;
  uint64_t next_id{};
  std::shared_ptr<rclcpp::TimerBase> timer_;
  rclcpp::Publisher<perf_tool_msgs::msg::Bytes>::SharedPtr pub_;
  ClientResults results_;
  mutable std::mutex results_mutex_;
  rclcpp::Serialization<perf_tool_msgs::msg::Bytes> serializer_;
};

template<typename NodeT>
class NodeRunner {
public:
  NodeRunner()
  {
    if (!rclcpp::signal_handlers_installed()) {
      rclcpp::install_signal_handlers();
    }
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
  }

  template<typename... NodeArgs>
  void
  start(
    rmw_qos_profile_t rmw_qos,
    std::chrono::nanoseconds experiment_duration,
    NodeArgs... args)
  {
    rclcpp::NodeOptions no;
    no.context(context_);
    rclcpp::QoS pub_qos{{rmw_qos.history, rmw_qos.depth}, rmw_qos};
    node_ = std::make_shared<NodeT>(args... , pub_qos, no);

    running_thread_ = std::thread([this, experiment_duration]() {
      auto start = std::chrono::system_clock::now();
      rclcpp::ExecutorOptions eo;
      eo.context = context_;
      rclcpp::executors::SingleThreadedExecutor ex{eo};
      ex.add_node(node_);

      auto now = start;
      while (now < start + experiment_duration && context_->is_valid()) {
        ex.spin_once(start + experiment_duration - now);
        now = std::chrono::system_clock::now();
      }
    });
  }

  void stop()
  {
    context_->shutdown("stopping perf node runner");
  }

  void join()
  {
    if (running_thread_.joinable()) {
      running_thread_.join();
    }
  }

  auto get_node()
  {
    if (!node_) {
      throw std::runtime_error{"perf node runner was not started"};
    }
    return node_;
  }
private:
  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<NodeT> node_;
  std::thread running_thread_;
};
}  // namespace perf_tool

PYBIND11_MODULE(perf_tool_impl, m) {
  m.doc() = "Python wrapper of perf tool implementation";

  pybind11::class_<perf_tool::ClientNode, std::shared_ptr<perf_tool::ClientNode>>(
    m, "ClientNode")
    .def("get_topic_name", &perf_tool::ClientNode::get_topic_name)
    .def("get_stringified_pub_gid", &perf_tool::ClientNode::get_stringified_pub_gid)
    .def("get_results", &perf_tool::ClientNode::get_results);
  pybind11::class_<perf_tool::ServerNode, std::shared_ptr<perf_tool::ServerNode>>(
    m, "ServerNode")
    .def("get_topic_name", &perf_tool::ServerNode::get_topic_name)
    .def("get_results", &perf_tool::ServerNode::get_results);
  using ClientRunner = perf_tool::NodeRunner<perf_tool::ClientNode>;
  pybind11::class_<ClientRunner>(
    m, "ClientRunner")
    .def(pybind11::init<>())
    .def("start", &ClientRunner::start<size_t, std::chrono::nanoseconds>)
    .def("stop", &ClientRunner::stop)
    .def("join", &ClientRunner::join)
    .def("get_node", &ClientRunner::get_node);
  using ServerRunner = perf_tool::NodeRunner<perf_tool::ServerNode>;
  pybind11::class_<ServerRunner>(
    m, "ServerRunner")
    .def(pybind11::init<>())
    .def("start", &ServerRunner::start<>)
    .def("stop", &ServerRunner::stop)
    .def("join", &ServerRunner::join)
    .def("get_node", &ServerRunner::get_node);
  pybind11::class_<perf_tool::ClientResults>(
    m, "ClientResults")
    .def_readwrite("message_ids", &perf_tool::ClientResults::message_ids)
    .def_readwrite(
      "message_published_times",
      &perf_tool::ClientResults::message_published_times)
    .def_readwrite("message_sizes", &perf_tool::ClientResults::message_sizes);
  pybind11::class_<perf_tool::ServerResults>(
    m, "ServerResults")
    .def_readwrite("message_ids", &perf_tool::ServerResults::message_ids)
    .def_readwrite("message_latencies", &perf_tool::ServerResults::message_latencies)
    .def_readwrite("message_sizes", &perf_tool::ServerResults::message_sizes);
}
