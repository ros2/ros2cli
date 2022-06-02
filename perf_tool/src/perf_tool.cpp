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
#include "perf_tool_msgs/srv/get_results.hpp"

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

std::string
stringify_gid(const rmw_gid_t & rmw_gid)
{
  std::ostringstream oss;
  oss << std::hex;
  auto it = std::begin(rmw_gid.data);
  const auto last_elem_it = std::end(rmw_gid.data) - 1;
  for(; it != last_elem_it; ++it) {
    // need to cast to integer type, to avoid trying to use utf8 encoding
    oss << std::setw(2) << std::setfill('0') << static_cast<int>(*it) << ".";
  }
  oss << std::setw(2) << std::setfill('0') << static_cast<int>(*last_elem_it);
  return oss.str();
}

struct ServerCollectedInfo
{
  std::vector<std::chrono::time_point<std::chrono::system_clock>> message_reception_time;
  std::vector<size_t> message_ids;
  std::vector<std::chrono::nanoseconds> message_latencies;
  std::vector<size_t> message_sizes;
};

struct ServerResults
{
  perf_tool_msgs::srv::GetResults::Response statistics;
  ServerCollectedInfo collected_info;
};

class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(
    const rclcpp::QoS & sub_qos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : Node("server", "perf_tool", options)
  {
    sub_ = this->create_subscription<perf_tool_msgs::msg::Bytes>(
      "test_topic", sub_qos,
      std::bind(&ServerNode::handle_msg, this, std::placeholders::_1, std::placeholders::_2));
    srv_ = this->create_service<perf_tool_msgs::srv::GetResults>(
      "get_results",
      std::bind(&ServerNode::handle_get_results_request, this, std::placeholders::_1, std::placeholders::_2));
  }

  void handle_msg(
    const rclcpp::SerializedMessage & serialized_msg,
    const rclcpp::MessageInfo & msg_info)
  {
    auto rec_timestamp = std::chrono::system_clock::now();
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
      rec_timestamp =
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
      latency = rec_timestamp - pub_timestamp;
    }
    {
      std::lock_guard guard{pub_gid_to_collected_info_map_mutex_};
      RCLCPP_INFO_STREAM_ONCE(
        this->get_logger(),
        "Added one message with gid: " << stringify_gid(rmw_msg_info.publisher_gid));
      auto it_emplaced_pair = pub_gid_to_collected_info_map_.try_emplace(stringify_gid(rmw_msg_info.publisher_gid));
      auto & message_data = it_emplaced_pair.first->second;
      message_data.message_ids.emplace_back(msg.id);
      message_data.message_latencies.emplace_back(latency);
      message_data.message_sizes.emplace_back(received_bytes);
      message_data.message_reception_time.emplace_back(rec_timestamp);
    }
  }

  void
  handle_get_results_request(
    perf_tool_msgs::srv::GetResults::Request::SharedPtr req,
    perf_tool_msgs::srv::GetResults::Response::SharedPtr rep)
  {
    ServerCollectedInfo collected_info;
    {
      std::lock_guard guard{pub_gid_to_collected_info_map_mutex_};
      auto it = pub_gid_to_collected_info_map_.find(req->publisher_gid);
      if (it == pub_gid_to_collected_info_map_.end() || 0u == it->second.message_ids.size()) {
        return;
      }
      collected_info = std::move(it->second);
      // remove data from that publisher when the results are requested
      pub_gid_to_collected_info_map_.erase(it);
    }
    // calculate latency statistics
    const auto & latencies = collected_info.message_latencies;
    auto latency_avg_ms = std::accumulate(
      latencies.begin(),
      latencies.end(),
      0.0,
      [n_items = static_cast<double>(latencies.size())](auto lhs, auto rhs) {
        return lhs + static_cast<double>(rhs.count()) / 1e6 / n_items;
      });
    auto latency_var_ms2 = std::accumulate(
      latencies.begin(),
      latencies.end(),
      0.0,
      [latency_avg_ms, div = static_cast<double>(latencies.size() - 1)](auto lhs, auto rhs) {
        auto diff = static_cast<double>(rhs.count()) / 1e6 - latency_avg_ms;
        return lhs + diff * diff / div;
      });
    auto latency_min_max_its = std::minmax_element(latencies.begin(), latencies.end());
    rep->latency_avg_ms = latency_avg_ms;
    rep->latency_stdev_ms = std::sqrt(latency_var_ms2);
    rep->latency_min_ms = static_cast<double>(latency_min_max_its.first->count()) / 1e6;
    rep->latency_max_ms = static_cast<double>(latency_min_max_its.second->count()) / 1e6;

    // calculate total bytes transferred
    rep->total_bytes = std::accumulate(collected_info.message_sizes.begin(), collected_info.message_sizes.end(), 0);

    // messages lost and lost
    rep->messages_total = req->messages_total;
    rep->messages_lost = req->messages_total - collected_info.message_ids.size();

    // total experiment duration
    rep->experiment_duration_ns = (
      collected_info.message_reception_time.back() -
      collected_info.message_reception_time.front()).count();

    ServerResults results;
    results.collected_info = std::move(collected_info);
    results.statistics = *rep;
    {
      std::lock_guard guard{pub_gid_to_results_map_mutex_};
      pub_gid_to_results_map_.try_emplace(req->publisher_gid, results);
    }
    pub_gid_to_results_map_cv_.notify_one();
  }

  auto
  extract_results()
  {
    PubGidToResultsMap ret;
    {
      std::lock_guard guard{pub_gid_to_results_map_mutex_};
      ret = std::move(pub_gid_to_results_map_);
    }
    return ret;
  }

  void
  wait_for_results_available(std::chrono::nanoseconds timeout)
  {
    std::unique_lock guard{pub_gid_to_results_map_mutex_};
    pub_gid_to_results_map_cv_.wait_for(
      guard,
      timeout,
      [this]() {
        return pub_gid_to_results_map_.size() != 0u;
      });
  }

  std::string
  get_topic_name()
  {
    return sub_->get_topic_name();
  }

  void
  finalize_work(rclcpp::Executor &)
  {}
private:
  using PubGidToMessageData = std::unordered_map<std::string, ServerCollectedInfo>;
  PubGidToMessageData pub_gid_to_collected_info_map_;
  mutable std::mutex pub_gid_to_collected_info_map_mutex_;

  using PubGidToResultsMap = std::unordered_map<std::string, ServerResults>;
  PubGidToResultsMap pub_gid_to_results_map_;
  mutable std::mutex pub_gid_to_results_map_mutex_;
  mutable std::condition_variable pub_gid_to_results_map_cv_;

  rclcpp::Subscription<perf_tool_msgs::msg::Bytes>::SharedPtr sub_;
  rclcpp::Serialization<perf_tool_msgs::msg::Bytes> deserializer_;
  rclcpp::ServiceBase::SharedPtr srv_;
};

struct ClientCollectedInfo
{
  std::vector<size_t> message_ids;
  std::vector<std::chrono::time_point<std::chrono::system_clock>> message_published_times;
  std::vector<size_t> message_sizes;
};

struct ClientResults
{
  perf_tool_msgs::srv::GetResults::Response statistics;
  ClientCollectedInfo collected_info;
};

class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(
    size_t array_size,
    std::chrono::nanoseconds target_pub_period,
    const rclcpp::QoS & pub_qos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : Node("client", "perf_tool", options),
    array_size_{array_size},
    target_pub_period_{target_pub_period}
  {
    pub_ = this->create_publisher<perf_tool_msgs::msg::Bytes>("test_topic", pub_qos);
    // always keep the vector preallocated, to avoid delays when the timer is triggered
    bytes_.resize(array_size);
    client_ = this->create_client<perf_tool_msgs::srv::GetResults>("get_results");
  }

  bool wait_for_server(std::chrono::nanoseconds timeout = 5s)
  {
    auto start = std::chrono::system_clock::now();
    while (std::chrono::system_clock::now() < start + timeout) {
      if (client_->service_is_ready()) {
        return true;
      }
      RCLCPP_INFO(this->get_logger(), "perf server not available yet, waiting...");
      std::this_thread::sleep_for(500ms);
    }
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "perf server not available after waiting ["
      << std::chrono::duration_cast<std::chrono::seconds>(timeout).count() << "s]" << std::endl);
    return false;
  }

  void start_publishing()
  {
    timer_ = this->create_wall_timer(
      target_pub_period_,
      std::bind(&ClientNode::pub_next_msg, this));
  }

  void
  stop_publishing()
  {
    if (timer_) {
      timer_->cancel();
    }
  }

  void
  sync_with_server(rclcpp::Executor & exec)
  {
    if (!pub_->wait_for_all_acked(5s)) {
      RCLCPP_WARN(this->get_logger(), "Some messages were not acked by the perf server ...");
    }
    if (!client_->service_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "Perf server is not available anymore, cannot get the results back");
      return;
    }
    auto req = std::make_shared<perf_tool_msgs::srv::GetResults::Request>();
    req->publisher_gid = this->get_stringified_pub_gid();
    req->messages_total = this->collected_info_.message_ids.size();
    auto future = client_->async_send_request(req);
    exec.spin_until_future_complete(future);
    statistics_ = future.get();
  }

  ClientResults
  extract_results()
  {
    ClientResults results;
    if (!statistics_) {
      throw std::runtime_error{"you must first completely run the client before getting results"};
    }
    results.statistics = *statistics_;
    statistics_.reset();
    {
      std::lock_guard guard{collected_info_mutex_};
      results.collected_info = std::move(collected_info_);
    }
    return results;
  }

  std::string
  get_topic_name()
  {
    return pub_->get_topic_name();
  }

  std::string
  get_stringified_pub_gid()
  {
    return stringify_gid(pub_->get_gid());
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
      std::lock_guard guard{collected_info_mutex_};
      collected_info_.message_ids.emplace_back(msg.id);
      collected_info_.message_published_times.emplace_back(now);
      collected_info_.message_sizes.emplace_back(sent_bytes);
    }
    ++next_id;
  }

private:
  size_t array_size_;
  std::vector<unsigned char> bytes_;
  uint64_t next_id{};
  std::shared_ptr<rclcpp::TimerBase> timer_;
  rclcpp::Publisher<perf_tool_msgs::msg::Bytes>::SharedPtr pub_;
  rclcpp::Client<perf_tool_msgs::srv::GetResults>::SharedPtr client_;
  ClientCollectedInfo collected_info_;
  mutable std::mutex collected_info_mutex_;
  rclcpp::Serialization<perf_tool_msgs::msg::Bytes> serializer_;
  perf_tool_msgs::srv::GetResults::Response::SharedPtr statistics_;
  std::chrono::nanoseconds target_pub_period_;
};

auto
create_and_init_context()
{
  if (!rclcpp::signal_handlers_installed()) {
    rclcpp::install_signal_handlers();
  }
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  return context;
}

auto
executor_options_with_context(rclcpp::Context::SharedPtr context)
{
  rclcpp::ExecutorOptions eo;
  eo.context = std::move(context);
  return eo;
}

template<typename NodeT>
class NodeRunner {
public:
  template<typename... NodeArgs>
  NodeRunner(
    rmw_qos_profile_t rmw_qos,
    NodeArgs... args)
  : context_{create_and_init_context()},
    exec_{executor_options_with_context(context_)}
  {
    rclcpp::NodeOptions no;
    no.context(context_);
    rclcpp::QoS pub_qos{{rmw_qos.history, rmw_qos.depth}, rmw_qos};
    node_ = std::make_shared<NodeT>(args... , pub_qos, no);
    exec_.add_node(node_);
  }

  void
  start(std::chrono::nanoseconds duration)
  {
    running_thread_ = std::thread([
      this, duration, stop_token = thread_stop_promise_.get_future()]()
      {
        auto start = std::chrono::system_clock::now();
        auto now = start;
        while (
          (duration < 0ns || now <= start + duration) &&
          stop_token.wait_for(0s) != std::future_status::ready &&
          context_->is_valid())
        {
          exec_.spin_once(start + duration - now);
          now = std::chrono::system_clock::now();
        }
      });
  }

  void stop()
  {
    thread_stop_promise_.set_value();
  }

  void join()
  {
    if (running_thread_.joinable()) {
      running_thread_.join();
    }
  }

  auto
  get_node()
  {
    if (!node_) {
      throw std::runtime_error{"perf node runner was not started"};
    }
    return node_;
  }
protected:
  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<NodeT> node_;
  std::thread running_thread_;
  std::promise<void> thread_stop_promise_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

class ClientRunner : public NodeRunner<ClientNode>
{
public:
  using NodeRunner<ClientNode>::NodeRunner;
  void start(std::chrono::nanoseconds duration)
  {
    if (node_->wait_for_server()) {
      node_->start_publishing();
      return NodeRunner<ClientNode>::start(duration);
    }
    throw std::runtime_error{"failed to connect to server"};
  }

  void join()
  {
    bool sync_needed = running_thread_.joinable();
    NodeRunner<ClientNode>::join();
    if (sync_needed) {
      node_->stop_publishing();
      node_->sync_with_server(exec_);
    }
  }
};
}  // namespace perf_tool

PYBIND11_MODULE(perf_tool_impl, m) {
  m.doc() = "Python wrapper of perf tool implementation";
  using namespace perf_tool;

  pybind11::class_<ClientNode, std::shared_ptr<ClientNode>>(
    m, "ClientNode")
    .def("get_topic_name", &ClientNode::get_topic_name)
    .def("get_stringified_pub_gid", &ClientNode::get_stringified_pub_gid)
    .def("extract_results", &ClientNode::extract_results);
  pybind11::class_<ServerNode, std::shared_ptr<ServerNode>>(
    m, "ServerNode")
    .def("get_topic_name", &ServerNode::get_topic_name)
    .def("extract_results", &ServerNode::extract_results)
    .def("wait_for_results_available", &ServerNode::wait_for_results_available);
  pybind11::class_<ClientRunner>(
    m, "ClientRunner")
    .def(pybind11::init<rmw_qos_profile_t, size_t, std::chrono::nanoseconds>())
    .def("start", &ClientRunner::start)
    .def("stop", &ClientRunner::stop)
    .def("join", &ClientRunner::join)
    .def("get_node", &ClientRunner::get_node);
  using ServerRunner = NodeRunner<ServerNode>;
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
  using Statistics = perf_tool_msgs::srv::GetResults::Response;
  pybind11::class_<perf_tool_msgs::srv::GetResults::Response>(
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
