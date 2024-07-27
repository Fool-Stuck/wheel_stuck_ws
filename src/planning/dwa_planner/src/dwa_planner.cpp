// Copyright 2024 Fool Stuck Engineers
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

#include "dwa_planner/dwa_planner.hpp"

#include <wheel_stuck_utils/math/math.hpp>

#include <limits>

namespace dwa_planner
{

DWAPlanner::State::State(
  const double x, const double y, const double yaw, const double velocity,
  const double angular_velocity)
: x(x), y(y), yaw(yaw), velocity(velocity), angular_velocity(angular_velocity)
{
}

void DWAPlanner::State::simulate(
  const double velocity, const double angular_velocity, const double dt)
{
  x += velocity * std::cos(yaw) * dt;
  y += velocity * std::sin(yaw) * dt;
  yaw += angular_velocity * dt;
  this->velocity = velocity;
  this->angular_velocity = angular_velocity;
}

DWAPlanner::Window::Window()
: min_velocity(0.0), max_velocity(0.0), min_angular_velocity(0.0), max_angular_velocity(0.0)
{
}

DWAPlanner::Window::Window(
  const double min_velocity, const double max_velocity, const double min_angular_velocity,
  const double max_angular_velocity)
: min_velocity(min_velocity),
  max_velocity(max_velocity),
  min_angular_velocity(min_angular_velocity),
  max_angular_velocity(max_angular_velocity)
{
}

DWAPlanner::DWAPlanner(const rclcpp::NodeOptions & options) : Node("dwa_planner", options)
{
  // Declare Parameters
  double update_rate_hz = this->declare_parameter("update_rate_hz", 20.0);
  dt_ = 1.0 / update_rate_hz;

  map_timeout_ = this->declare_parameter("map_timeout", 1.0);
  odom_timeout_ = this->declare_parameter("odom_timeout", 1.0);
  goal_timeout_ = this->declare_parameter("goal_timeout", 1.0);

  min_velocity_ = this->declare_parameter("min_velocity", 0.0);
  max_velocity_ = this->declare_parameter("max_velocity", 0.0);
  max_acceleration_ = this->declare_parameter("max_acceleration", 0.0);
  max_angular_speed_ = this->declare_parameter("max_angular_speed", 0.0);
  max_angular_acceleration_ = this->declare_parameter("max_angular_acceleration", 0.0);

  prediction_time_ = this->declare_parameter("prediction_time", 1.0);
  velocity_resolution_ = this->declare_parameter("velocity_resolution", 10);
  angular_velocity_resolution_ = this->declare_parameter("angular_velocity_resolution", 10);
  velocity_resolution_inv_ = 1.0 / velocity_resolution_;
  angular_velocity_resolution_inv_ = 1.0 / angular_velocity_resolution_;

  // Declare Subscriptions
  {
    map_sub_ = OccupancyGridSubscription::create_subscription(this, "~/input/local_costmap");
    odom_sub_ = OdometrySubscription::create_subscription(this, "~/input/odom");
    goal_sub_ = PoseStampedSubscription::create_subscription(this, "~/input/local_goal");
  }

  // Declare Publishers
  {
    cmd_pub_ = this->create_publisher<TwistStamped>("~/output/cmd_vel", 1);
  }

  // Declare timer for update function
  {
    auto update_callback = [this]() { this->update(); };
    auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt_));
    update_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(update_callback)>>(
      this->get_clock(), period, std::move(update_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(update_timer_, nullptr);
  }
}

void DWAPlanner::update()
{
  if (!subscribe_and_validate()) return;
  Trajectory best_trajectory = planning();

  TwistStamped cmd;
  {
    cmd.header.frame_id = odom_->child_frame_id;
    cmd.header.stamp = this->now();
    cmd.twist.linear.x = best_trajectory.front().velocity;
    cmd.twist.angular.z = best_trajectory.front().angular_velocity;
  }

  cmd_pub_->publish(cmd);
}

bool DWAPlanner::subscribe_and_validate()
{
  bool isPlannerExecutable = true;

  // Subscribe map
  {
    if (try_subscribe_map()) last_map_sub_time_ = this->now();
    if (!map_ || (this->now() - last_map_sub_time_).seconds() > map_timeout_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No map received");
      isPlannerExecutable = false;
    }
  }

  // Subscribe odom
  {
    if (try_subscribe_odom()) last_odom_sub_time_ = this->now();
    if (!odom_ || (this->now() - last_odom_sub_time_).seconds() > odom_timeout_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No odom received");
      isPlannerExecutable = false;
    }
  }

  // Subscribe goal
  {
    if (try_subscribe_goal()) last_goal_sub_time_ = this->now();
    if (!goal_ || (this->now() - last_goal_sub_time_).seconds() > goal_timeout_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No goal received");
      isPlannerExecutable = false;
    }
  }

  if (!isPlannerExecutable) return false;

  if (map_->header.frame_id != odom_->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Map and odom frame_id mismatch");
    isPlannerExecutable = false;
  }

  if (goal_->header.frame_id != odom_->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Goal and odom frame_id mismatch");
    isPlannerExecutable = false;
  }

  return isPlannerExecutable;
}

bool DWAPlanner::try_subscribe_map()
{
  auto map_msg = map_sub_->get_data();
  if (!map_msg) return false;
  map_ = map_msg;
  return true;
}

bool DWAPlanner::try_subscribe_odom()
{
  auto odom_msg = odom_sub_->get_data();
  if (!odom_msg) return false;
  odom_ = odom_msg;
  return true;
}

bool DWAPlanner::try_subscribe_goal()
{
  auto goal_msg = goal_sub_->get_data();
  if (!goal_msg) return false;
  goal_ = goal_msg;
  return true;
}

DWAPlanner::Trajectory DWAPlanner::planning()
{
  Twist twist = odom_->twist.twist;
  double current_velocity = twist.linear.x;
  double current_angular_velocity = twist.angular.z;

  Window window = generate_window(twist);

  double min_cost = std::numeric_limits<double>::max();
  Trajectory best_trajectory;

  for (int v = 0; v < velocity_resolution_; v++) {
    double velocity = wheel_stuck_utils::math::lerp(
      window.min_velocity, window.max_velocity, v * velocity_resolution_inv_);
    for (int a = 0; a < angular_velocity_resolution_; a++) {
      double angular_velocity = wheel_stuck_utils::math::lerp(
        window.min_angular_velocity, window.max_angular_velocity,
        a * angular_velocity_resolution_inv_);

      double cost = 0.0;
      State state(0.0, 0.0, 0.0, current_velocity, current_angular_velocity);
      Trajectory trajectory;
      for (float t = 0.0; t <= prediction_time_; t += dt_) {
        state.simulate(velocity, angular_velocity, dt_);
        cost += calculate_stage_cost(state);
        if (cost > min_cost) break;
        trajectory.push_back(state);
      }

      cost += calculate_terminal_cost(state);
      if (cost > min_cost) continue;

      min_cost = cost;
      best_trajectory = trajectory;
    }
  }

  if (best_trajectory.empty()) {
    best_trajectory.push_back(State(0.0, 0.0, 0.0, window.min_velocity, 0.0));
  }
  return best_trajectory;
}

DWAPlanner::Window DWAPlanner::generate_window(const Twist current_twist)
{
  double current_velocity = current_twist.linear.x;
  double current_angular_velocity = current_twist.angular.z;
  Window window;
  {
    window.min_velocity = std::max(current_velocity - max_acceleration_ * dt_, min_velocity_);
    window.max_velocity = std::min(current_velocity + max_acceleration_ * dt_, max_velocity_);
    window.min_angular_velocity =
      std::max(current_angular_velocity - max_angular_acceleration_ * dt_, -max_angular_speed_);
    window.max_angular_velocity =
      std::min(current_angular_velocity + max_angular_acceleration_ * dt_, max_angular_speed_);
  }
  return window;
}

double DWAPlanner::calculate_stage_cost(const State & state)
{
  return 0.0;
}

double DWAPlanner::calculate_terminal_cost(const State & state)
{
  return 0.0;
}

}  // namespace dwa_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dwa_planner::DWAPlanner)
