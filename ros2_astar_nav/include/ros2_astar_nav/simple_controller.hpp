// Copyright 2024 The Authors
// Licensed under the Apache License, Version 2.0

#ifndef ROS2_ASTAR_NAV__SIMPLE_CONTROLLER_HPP_
#define ROS2_ASTAR_NAV__SIMPLE_CONTROLLER_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_astar_nav
{

/**
 * @brief Pure Pursuit controller for path following
 *
 * This class implements a Pure Pursuit controller that:
 * - Follows a planned path using lookahead distance
 * - Computes velocity commands for the robot
 * - Tracks progress along the path
 * - Checks for goal achievement
 */
class SimpleController
{
public:
  /**
   * @brief Constructor for SimpleController
   */
  SimpleController();

  /**
   * @brief Destructor for SimpleController
   */
  ~SimpleController() = default;

  /**
   * @brief Set controller parameters
   * @param max_linear_vel Maximum linear velocity (m/s)
   * @param max_angular_vel Maximum angular velocity (rad/s)
   * @param lookahead_distance Distance to lookahead point (m)
   * @param goal_pos_tolerance Position tolerance for goal (m)
   * @param goal_orient_tolerance Orientation tolerance for goal (rad)
   */
  void setParameters(
    double max_linear_vel,
    double max_angular_vel,
    double lookahead_distance,
    double goal_pos_tolerance,
    double goal_orient_tolerance);

  /**
   * @brief Set the path to follow
   * @param path Navigation path
   */
  void setPath(const nav_msgs::msg::Path & path);

  /**
   * @brief Compute velocity command based on current pose and path
   * @param current_pose Current robot pose
   * @param cmd_vel Output velocity command
   * @return true if command computed successfully, false if goal reached or error
   */
  bool computeVelocity(
    const geometry_msgs::msg::PoseStamped & current_pose,
    geometry_msgs::msg::Twist & cmd_vel);

  /**
   * @brief Check if goal has been reached
   * @param current_pose Current robot pose
   * @return true if goal reached within tolerances
   */
  bool isGoalReached(const geometry_msgs::msg::PoseStamped & current_pose) const;

  /**
   * @brief Get the distance remaining to goal
   * @param current_pose Current robot pose
   * @return Distance to goal in meters
   */
  double getDistanceToGoal(const geometry_msgs::msg::PoseStamped & current_pose) const;

  /**
   * @brief Get current path index
   * @return Current index in path
   */
  size_t getCurrentPathIndex() const;

  /**
   * @brief Reset the controller state
   */
  void reset();

  /**
   * @brief Check if controller has a valid path
   * @return true if path is set and valid
   */
  bool hasPath() const;

private:
  /**
   * @brief Find the lookahead point on the path
   * @param current_pose Current robot pose
   * @param lookahead_point Output lookahead point
   * @return true if point found
   */
  bool findLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    geometry_msgs::msg::PoseStamped & lookahead_point);

  /**
   * @brief Calculate distance between two poses
   * @param pose1 First pose
   * @param pose2 Second pose
   * @return Euclidean distance
   */
  double calculateDistance(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2) const;

  /**
   * @brief Calculate the angle difference between current heading and target
   * @param current_pose Current robot pose
   * @param target Target pose
   * @return Angle difference in radians
   */
  double calculateAngleDifference(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & target) const;

  /**
   * @brief Get yaw angle from quaternion
   * @param pose Pose with quaternion orientation
   * @return Yaw angle in radians
   */
  double getYaw(const geometry_msgs::msg::PoseStamped & pose) const;

  /**
   * @brief Normalize angle to [-pi, pi]
   * @param angle Input angle
   * @return Normalized angle
   */
  double normalizeAngle(double angle) const;

  /**
   * @brief Find closest point on path to robot
   * @param current_pose Current robot pose
   * @return Index of closest point
   */
  size_t findClosestPointIndex(const geometry_msgs::msg::PoseStamped & current_pose) const;

  // Path data
  nav_msgs::msg::Path path_;
  std::mutex path_mutex_;
  size_t current_path_index_;
  bool has_path_;

  // Controller parameters
  double max_linear_velocity_;
  double max_angular_velocity_;
  double lookahead_distance_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;

  // State tracking
  bool goal_reached_;
};

}  // namespace ros2_astar_nav

#endif  // ROS2_ASTAR_NAV__SIMPLE_CONTROLLER_HPP_
