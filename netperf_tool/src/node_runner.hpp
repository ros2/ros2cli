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

#ifndef NODE_RUNNER_HPP_
#define NODE_RUNNER_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include "rclcpp/context.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

#include "client_node.hpp"
#include "server_node.hpp"

namespace netperf_tool
{

/// Create a rclcpp::Context and init it.
rclcpp::Context::SharedPtr
create_and_init_context();

/// Return a rclcpp::ExecutorOptions using the provided context.
rclcpp::ExecutorOptions
executor_options_with_context(rclcpp::Context::SharedPtr context);

/// Class that allow easily spinning a node in an executor asynchronously.
/**
 * \tparam NodeT Class extending rclcpp::Node.
 */
template<typename NodeT>
class NodeRunner
{
public:
  /// Construct.
  /**
   * \param[in] rmw_qos Publisher or subscription qos profile, that will be
   *  passed to the node constructor.
   * \param[in] args Extra arguments required by NodeT constructor.
   */
  template<typename ... NodeArgs>
  NodeRunner(
    rmw_qos_profile_t rmw_qos,
    NodeArgs... args)
  : context_{create_and_init_context()},
    exec_{executor_options_with_context(context_)}
  {
    rclcpp::NodeOptions no;
    no.context(context_);
    rclcpp::QoS qos{{rmw_qos.history, rmw_qos.depth}, rmw_qos};
    node_ = std::make_shared<NodeT>(no, qos, args ...);
    exec_.add_node(node_);
  }

  /// Start to spin asynchronously.
  /**
   * \param[in] duration Total time to spin.
   */
  void
  start(std::chrono::nanoseconds duration)
  {
    running_thread_ = std::thread(
      [
        this, duration, stop_token = thread_stop_promise_.get_future()]()
      {
        auto start = std::chrono::system_clock::now();
        auto now = start;
        while (
          (duration < std::chrono::nanoseconds{0} || now <= start + duration) &&
          stop_token.wait_for(std::chrono::seconds{0}) != std::future_status::ready &&
          context_->is_valid())
        {
          exec_.spin_once(start + duration - now);
          now = std::chrono::system_clock::now();
        }
      });
  }

  /// Signal the spinning thread to stop.
  /**
   * The spinning thread will not stop immediately.
   * Use NodeT::join() after to guarantee the spinning thread has finished.
   */
  void stop()
  {
    thread_stop_promise_.set_value();
  }

  /// Join the spinning thread.
  void join()
  {
    if (running_thread_.joinable()) {
      running_thread_.join();
    }
  }

  /// Get the created NodeT.
  auto
  get_node()
  {
    if (!node_) {
      throw std::runtime_error{"perf node runner was not started"};
    }
    return node_;
  }

protected:
  /// Context used to create the node and executor.
  rclcpp::Context::SharedPtr context_;
  /// The node being spinned.
  std::shared_ptr<NodeT> node_;
  /// Spinning thread.
  std::thread running_thread_;
  /// Promise used to indicate the spinning thread to stop running.
  std::promise<void> thread_stop_promise_;
  /// Executor used to spin.
  rclcpp::executors::SingleThreadedExecutor exec_;
};

/// Extension of NodeRunner for ClientNode.
class ClientRunner : public NodeRunner<ClientNode>
{
public:
  using NodeRunner<ClientNode>::NodeRunner;
  void start(std::chrono::nanoseconds duration);
  void join();
};
}  // namespace netperf_tool

#endif  // NODE_RUNNER_HPP_
