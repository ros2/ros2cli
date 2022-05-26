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

class Server : public rclcpp::Node
{
public:
  explicit Server(
    const rclcpp::QoS & sub_qos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : Node("perf_tool_server", options)
  {
    sub_ = this->create_subscription<perf_tool_msgs::msg::Bytes>(
      "perf_topic", sub_qos,
      std::bind(&Server::handle_msg, this, std::placeholders::_1, std::placeholders::_2));
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
    results_.message_ids.emplace_back(msg.id);
    results_.message_latencies.emplace_back(latency);
    results_.message_sizes.emplace_back(received_bytes);
  }

  ServerResults
  get_results() const
  {
    return results_;
  }

private:
  ServerResults results_;
  rclcpp::Subscription<perf_tool_msgs::msg::Bytes>::SharedPtr sub_;
  rclcpp::Serialization<perf_tool_msgs::msg::Bytes> deserializer_;
};

ServerResults
run_server(rmw_qos_profile_t rmw_sub_qos, std::chrono::nanoseconds experiment_duration)
{
  if(!rclcpp::signal_handlers_installed()) {
    rclcpp::install_signal_handlers();
  }
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions no;
  no.context(context);
  rclcpp::QoS sub_qos{{rmw_sub_qos.history, rmw_sub_qos.depth}, rmw_sub_qos};
  auto perf_server = std::make_shared<Server>(sub_qos, no);
  rclcpp::ExecutorOptions eo;
  eo.context = context;
  rclcpp::executors::SingleThreadedExecutor ex{eo};
  ex.add_node(perf_server);

  auto start = std::chrono::system_clock::now();
  auto now = start;
  pybind11::gil_scoped_release gil_guard;
  while (now < start + experiment_duration && context->is_valid()) {
    ex.spin_once(start + experiment_duration - now);
    now = std::chrono::system_clock::now();
  }
  return perf_server->get_results();
}

struct ClientResults
{
  std::vector<size_t> message_ids;
  std::vector<std::chrono::time_point<std::chrono::system_clock>> message_published_times;
  std::vector<size_t> message_sizes;
};

class Client : public rclcpp::Node
{
public:
  explicit Client(
    size_t array_size,
    std::chrono::nanoseconds target_pub_period,
    const rclcpp::QoS & pub_qos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : Node("perf_tool_client", options), array_size_{array_size}
  {
    pub_ = this->create_publisher<perf_tool_msgs::msg::Bytes>("perf_topic", pub_qos);
    timer_ = this->create_wall_timer(
      target_pub_period,
      std::bind(&Client::pub_next_msg, this));
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
    results_.message_ids.emplace_back(msg.id);
    results_.message_published_times.emplace_back(now);
    results_.message_sizes.emplace_back(sent_bytes);
    ++next_id;
  }

  ClientResults
  get_results() {
    return results_;
  }

private:
  size_t array_size_;
  std::vector<unsigned char> bytes_;
  uint64_t next_id{};
  std::shared_ptr<rclcpp::TimerBase> timer_;
  rclcpp::Publisher<perf_tool_msgs::msg::Bytes>::SharedPtr pub_;
  ClientResults results_;
  rclcpp::Serialization<perf_tool_msgs::msg::Bytes> serializer_;
};

ClientResults
run_client(
  size_t array_size,
  std::chrono::nanoseconds target_pub_period,
  rmw_qos_profile_t rmw_pub_qos,
  std::chrono::nanoseconds experiment_duration)
{
  std::cout << "\tarray_size: " << array_size << "\n\ttarget_pub_period: " << target_pub_period.count()
    << "\n\texperiment_duration: " << experiment_duration.count() << std::endl;
  if(!rclcpp::signal_handlers_installed()) {
    rclcpp::install_signal_handlers();
  }
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions no;
  no.context(context);
  auto start = std::chrono::system_clock::now();
  rclcpp::QoS pub_qos{{rmw_pub_qos.history, rmw_pub_qos.depth}, rmw_pub_qos};
  auto perf_client = std::make_shared<Client>(array_size, target_pub_period ,pub_qos, no);
  rclcpp::ExecutorOptions eo;
  eo.context = context;
  rclcpp::executors::SingleThreadedExecutor ex{eo};
  ex.add_node(perf_client);

  auto now = start;
  pybind11::gil_scoped_release gil_guard;
  while (now < start + experiment_duration && context->is_valid()) {
    ex.spin_once(start + experiment_duration - now);
    now = std::chrono::system_clock::now();
  }
  return perf_client->get_results();
}
}  // namespace perf_tool

PYBIND11_MODULE(perf_tool_impl, m) {
  m.doc() = "Python wrapper of perf tool implementation";

  m.def("run_server", &perf_tool::run_server);
  m.def("run_client", &perf_tool::run_client);
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
