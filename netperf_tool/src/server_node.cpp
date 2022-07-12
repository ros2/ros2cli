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

#include "server_node.hpp"

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"

#include "utils.hpp"

namespace netperf_tool
{

ServerNode::ServerNode(
  const rclcpp::NodeOptions & options,
  const rclcpp::QoS & sub_qos)
: Node("server", "netperf_tool", options)
{
  sub_ = this->create_subscription<netperf_tool_interfaces::msg::Bytes>(
    "test_topic", sub_qos,
    std::bind(&ServerNode::handle_msg, this, std::placeholders::_1, std::placeholders::_2));
  srv_ = this->create_service<netperf_tool_interfaces::srv::GetResults>(
    "get_results",
    std::bind(
      &ServerNode::handle_get_results_request,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

void
ServerNode::handle_msg(
  const rclcpp::SerializedMessage & serialized_msg,
  const rclcpp::MessageInfo & msg_info)
{
  auto rec_timestamp = system_time_clock_.now();
  std::chrono::nanoseconds latency;
  size_t received_bytes = serialized_msg.size();
  netperf_tool_interfaces::msg::Bytes msg;
  deserializer_.deserialize_message(&serialized_msg, &msg);
  const auto & rmw_msg_info = msg_info.get_rmw_message_info();
  if (rmw_msg_info.source_timestamp != 0 && rmw_msg_info.received_timestamp != 0) {
    // source and received timestamp supported
    // TODO(ivanpauno): Add a way to evaluate this in rmw_feature_supported() to avoid the if().
    RCLCPP_INFO_ONCE(
      this->get_logger(),
      "source and received timestamp supported, using them to calculate latency!");
    auto pub_timestamp = rclcpp::Time{rmw_msg_info.source_timestamp};
    rec_timestamp = rclcpp::Time{rmw_msg_info.received_timestamp};
    latency = std::chrono::nanoseconds{(rec_timestamp - pub_timestamp).nanoseconds()};
  } else {
    // TODO(ivanpauno): Maybe it's worth to always calculate this latency (?)
    // as it takes in account the executor overhead.
    RCLCPP_INFO_ONCE(
      this->get_logger(),
      "either source or received timestamp are not supported,"
      "using timestamps added to the message to calculate latency");
    rclcpp::Time pub_timestamp = msg.timestamp;
    latency = std::chrono::nanoseconds{(rec_timestamp - pub_timestamp).nanoseconds()};
  }
  bool emplaced{false};
  auto string_gid = stringify_gid(rmw_msg_info.publisher_gid);
  {
    // Store message info
    std::lock_guard guard{pub_gid_to_collected_info_map_mutex_};
    RCLCPP_INFO_STREAM_ONCE(
      this->get_logger(),
      "Added one message with gid: " << string_gid);
    auto it_emplaced_pair = pub_gid_to_collected_info_map_.try_emplace(string_gid);
    auto & collected_info = it_emplaced_pair.first->second;
    ServerMessageInfo info;
    info.id = msg.id;
    info.latency = latency;
    info.serialized_size = received_bytes;
    info.reception_time = rec_timestamp;
    collected_info.message_infos.emplace_back(info);
    emplaced = it_emplaced_pair.second;
  }
  if (emplaced) {
    // first time
    std::lock_guard guard{notify_mutex_};
    new_clients_.emplace_back(string_gid);
  }
  notify_cv_.notify_one();
}

void
ServerNode::handle_get_results_request(
  netperf_tool_interfaces::srv::GetResults::Request::SharedPtr req,
  netperf_tool_interfaces::srv::GetResults::Response::SharedPtr rep)
{
  ServerCollectedInfo collected_info;
  {
    std::lock_guard guard{pub_gid_to_collected_info_map_mutex_};
    auto it = pub_gid_to_collected_info_map_.find(req->publisher_gid);
    if (it == pub_gid_to_collected_info_map_.end() || 0u == it->second.message_infos.size()) {
      return;
    }
    collected_info = std::move(it->second);
    // remove data from that publisher when the results are requested
    pub_gid_to_collected_info_map_.erase(it);
  }
  // calculate latency statistics
  auto latency_avg_ms = std::accumulate(
    collected_info.message_infos.begin(),
    collected_info.message_infos.end(),
    0.0,
    [n_items = static_cast<double>(collected_info.message_infos.size())](auto lhs, auto rhs) {
      return lhs + static_cast<double>(rhs.latency.count()) / 1e6 / n_items;
    });
  auto latency_var_ms2 = std::accumulate(
    collected_info.message_infos.begin(),
    collected_info.message_infos.end(),
    0.0,
    [latency_avg_ms, div = static_cast<double>(collected_info.message_infos.size() - 1)](
      auto lhs, auto rhs)
    {
      auto diff = static_cast<double>(rhs.latency.count()) / 1e6 - latency_avg_ms;
      return lhs + diff * diff / div;
    });
  auto latency_min_max_its = std::minmax_element(
    collected_info.message_infos.begin(),
    collected_info.message_infos.end(),
    [](auto lhs, auto rhs) {
      return lhs.latency < rhs.latency;
    });
  rep->latency_avg_ms = latency_avg_ms;
  rep->latency_stdev_ms = std::sqrt(latency_var_ms2);
  rep->latency_min_ms = static_cast<double>(latency_min_max_its.first->latency.count()) / 1e6;
  rep->latency_max_ms = static_cast<double>(latency_min_max_its.second->latency.count()) / 1e6;

  // calculate total bytes transferred
  rep->total_bytes = std::accumulate(
    collected_info.message_infos.begin(),
    collected_info.message_infos.end(),
    0u,
    [](auto lhs, auto rhs) {
      return lhs + rhs.serialized_size;
    });

  // messages lost and lost
  rep->messages_total = req->messages_total;
  rep->messages_lost = req->messages_total - collected_info.message_infos.size();

  // total experiment duration
  rep->experiment_duration_ns = static_cast<size_t>(
    (
      collected_info.message_infos.back().reception_time -
      collected_info.message_infos.front().reception_time
    ).nanoseconds()
  );

  ServerResults results;
  results.collected_info = std::move(collected_info);
  results.statistics = *rep;
  {
    std::lock_guard guard{notify_mutex_};
    pub_gid_to_results_map_.try_emplace(req->publisher_gid, results);
  }
  notify_cv_.notify_one();
}

ServerNode::PubGidToResultsMap
ServerNode::extract_results()
{
  PubGidToResultsMap ret;
  {
    std::lock_guard guard{notify_mutex_};
    ret = std::move(pub_gid_to_results_map_);
  }
  return ret;
}

std::vector<std::string>
ServerNode::extract_new_clients()
{
  std::vector<std::string> ret;
  {
    std::lock_guard guard{notify_mutex_};
    ret = std::move(new_clients_);
  }
  return ret;
}

void
ServerNode::wait_for_results_available(std::chrono::nanoseconds timeout)
{
  std::unique_lock guard{notify_mutex_};
  notify_cv_.wait_for(
    guard,
    timeout,
    [this]() {
      return pub_gid_to_results_map_.size() != 0u || new_clients_.size() != 0u;
    });
}

std::string
ServerNode::get_topic_name()
{
  return sub_->get_topic_name();
}
}  // namespace netperf_tool
