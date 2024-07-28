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

#ifndef DWA_PLANNER__DWA_PLANNER_HPP_
#define DWA_PLANNER__DWA_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_common_utils/ros/no_callback_subscription.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>

namespace dwa_planner
{

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Odometry = nav_msgs::msg::Odometry;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Twist = geometry_msgs::msg::Twist;
using TwistStamped = geometry_msgs::msg::TwistStamped;

using OccupancyGridSubscription =
  wheel_stuck_common_utils::ros::NoCallbackSubscription<OccupancyGrid>;
using OdometrySubscription = wheel_stuck_common_utils::ros::NoCallbackSubscription<Odometry>;
using PoseStampedSubscription = wheel_stuck_common_utils::ros::NoCallbackSubscription<PoseStamped>;
using TwistStampedPublisher = rclcpp::Publisher<TwistStamped>;

class DWAPlanner : public rclcpp::Node
{
private:
  class State
  {
  public:
    State(
      const double x, const double y, const double yaw, const double velocity,
      const double angular_velocity);
    void simulate(const double velocity, const double angular_velocity, const double dt);
    double x;
    double y;
    double yaw;
    double velocity;
    double angular_velocity;
  };

  class Window
  {
  public:
    Window();
    Window(
      const double min_velocity, const double max_velocity, const double min_angular_velocity,
      const double max_angular_velocity);
    double min_velocity;
    double max_velocity;
    double min_angular_velocity;
    double max_angular_velocity;
  };

  using Trajectory = std::vector<State>;

public:
  explicit DWAPlanner(const rclcpp::NodeOptions & options);

private:
  void update();
  bool subscribe_and_validate();
  bool try_subscribe_map();
  bool try_subscribe_odom();
  bool try_subscribe_goal();

  Trajectory planning();
  Window generate_window(const Twist current_twist);
  double calculate_stage_cost(const State & state);
  double calculate_terminal_cost(const State & state);

  rclcpp::TimerBase::SharedPtr update_timer_;

  OccupancyGridSubscription::SharedPtr map_sub_;
  OdometrySubscription::SharedPtr odom_sub_;
  PoseStampedSubscription::SharedPtr goal_sub_;
  TwistStampedPublisher::SharedPtr cmd_pub_;

  OccupancyGrid::ConstSharedPtr map_;
  Odometry::ConstSharedPtr odom_;
  PoseStamped::ConstSharedPtr goal_;

  rclcpp::Time last_map_sub_time_;
  rclcpp::Time last_odom_sub_time_;
  rclcpp::Time last_goal_sub_time_;

  double dt_;
  double map_timeout_;
  double odom_timeout_;
  double goal_timeout_;

  double min_velocity_;
  double max_velocity_;
  double max_angular_speed_;

  double max_acceleration_;
  double max_angular_acceleration_;

  double prediction_time_;
  int velocity_resolution_;
  int angular_velocity_resolution_;

  double velocity_resolution_inv_;
  double angular_velocity_resolution_inv_;
};
}  // namespace dwa_planner

#endif  // DWA_PLANNER__DWA_PLANNER_HPP_
