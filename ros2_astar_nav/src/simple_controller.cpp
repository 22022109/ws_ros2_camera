// Copyright 2024 The Authors
// Licensed under the Apache License, Version 2.0

#include "ros2_astar_nav/simple_controller.hpp"

#include <algorithm>
#include <cmath>

namespace ros2_astar_nav
{

SimpleController::SimpleController()
: current_path_index_(0),
  has_path_(false),
  max_linear_velocity_(0.5),
  max_angular_velocity_(1.0),
  lookahead_distance_(0.5),
  goal_position_tolerance_(0.1),
  goal_orientation_tolerance_(0.1),
  goal_reached_(false)
{
}

void SimpleController::setParameters(
  double max_linear_vel,
  double max_angular_vel,
  double lookahead_distance,
  double goal_pos_tolerance,
  double goal_orient_tolerance)
{
  max_linear_velocity_ = max_linear_vel;
  max_angular_velocity_ = max_angular_vel;
  lookahead_distance_ = lookahead_distance;
  goal_position_tolerance_ = goal_pos_tolerance;
  goal_orientation_tolerance_ = goal_orient_tolerance;
}

void SimpleController::setPath(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  path_ = path;
  current_path_index_ = 0;
  has_path_ = !path.poses.empty();
  goal_reached_ = false;
}

bool SimpleController::computeVelocity(
  const geometry_msgs::msg::PoseStamped & current_pose,
  geometry_msgs::msg::Twist & cmd_vel)
{
  std::lock_guard<std::mutex> lock(path_mutex_);

  // Initialize velocity to zero
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;

  if (!has_path_ || path_.poses.empty()) {
    return false;
  }

  // Check if goal is reached
  if (isGoalReached(current_pose)) {
    goal_reached_ = true;
    return false;  // Signal that we're done
  }

  // Find lookahead point
  geometry_msgs::msg::PoseStamped lookahead_point;
  if (!findLookaheadPoint(current_pose, lookahead_point)) {
    // Can't find lookahead point, try to reach the goal directly
    lookahead_point = path_.poses.back();
  }

  // Calculate angle to lookahead point
  double angle_to_target = calculateAngleDifference(current_pose, lookahead_point);

  // Calculate distance to lookahead point
  double distance_to_target = calculateDistance(current_pose, lookahead_point);

  // Pure Pursuit controller
  // Angular velocity: ω = 2 * v * sin(α) / L
  // where α is angle to lookahead point and L is lookahead distance

  // Determine linear velocity based on angle error
  // Reduce speed when turning sharply
  double angle_factor = std::cos(angle_to_target);
  angle_factor = std::max(0.0, angle_factor);  // Only positive factor

  double linear_vel = max_linear_velocity_ * angle_factor;

  // Further reduce speed when close to goal
  double goal_distance = getDistanceToGoal(current_pose);
  if (goal_distance < lookahead_distance_) {
    linear_vel *= (goal_distance / lookahead_distance_);
    linear_vel = std::max(0.1, linear_vel);  // Minimum velocity to approach goal
  }

  // Calculate angular velocity using Pure Pursuit
  double angular_vel;
  if (distance_to_target > 0.01) {
    // Pure Pursuit curvature calculation
    double curvature = 2.0 * std::sin(angle_to_target) / distance_to_target;
    angular_vel = linear_vel * curvature;
  } else {
    // Very close to target, just rotate
    angular_vel = angle_to_target * 2.0;
    linear_vel = 0.0;
  }

  // If angle error is too large, rotate in place first
  if (std::abs(angle_to_target) > M_PI / 4) {
    linear_vel *= 0.3;  // Slow down significantly
    angular_vel = std::copysign(max_angular_velocity_ * 0.7, angle_to_target);
  }

  // Apply velocity limits
  cmd_vel.linear.x = std::clamp(linear_vel, -max_linear_velocity_, max_linear_velocity_);
  cmd_vel.angular.z = std::clamp(angular_vel, -max_angular_velocity_, max_angular_velocity_);

  return true;
}

bool SimpleController::isGoalReached(const geometry_msgs::msg::PoseStamped & current_pose) const
{
  if (path_.poses.empty()) {
    return false;
  }

  const auto & goal = path_.poses.back();

  // Check position tolerance
  double distance = calculateDistance(current_pose, goal);
  if (distance > goal_position_tolerance_) {
    return false;
  }

  // Check orientation tolerance
  double current_yaw = getYaw(current_pose);
  double goal_yaw = getYaw(goal);
  double yaw_diff = std::abs(normalizeAngle(current_yaw - goal_yaw));

  return yaw_diff <= goal_orientation_tolerance_;
}

double SimpleController::getDistanceToGoal(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  if (path_.poses.empty()) {
    return std::numeric_limits<double>::max();
  }

  return calculateDistance(current_pose, path_.poses.back());
}

size_t SimpleController::getCurrentPathIndex() const
{
  return current_path_index_;
}

void SimpleController::reset()
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  path_.poses.clear();
  current_path_index_ = 0;
  has_path_ = false;
  goal_reached_ = false;
}

bool SimpleController::hasPath() const
{
  return has_path_;
}

bool SimpleController::findLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  geometry_msgs::msg::PoseStamped & lookahead_point)
{
  if (path_.poses.empty()) {
    return false;
  }

  // Find closest point on path
  current_path_index_ = findClosestPointIndex(current_pose);

  // Search for lookahead point starting from current index
  double accumulated_distance = 0.0;

  for (size_t i = current_path_index_; i < path_.poses.size() - 1; ++i) {
    double segment_dist = calculateDistance(path_.poses[i], path_.poses[i + 1]);
    accumulated_distance += segment_dist;

    if (accumulated_distance >= lookahead_distance_) {
      // Interpolate to find exact lookahead point
      double overshoot = accumulated_distance - lookahead_distance_;
      double ratio = (segment_dist > 0.01) ? (segment_dist - overshoot) / segment_dist : 1.0;

      lookahead_point.header = path_.poses[i].header;
      lookahead_point.pose.position.x =
        path_.poses[i].pose.position.x +
        ratio * (path_.poses[i + 1].pose.position.x - path_.poses[i].pose.position.x);
      lookahead_point.pose.position.y =
        path_.poses[i].pose.position.y +
        ratio * (path_.poses[i + 1].pose.position.y - path_.poses[i].pose.position.y);
      lookahead_point.pose.position.z = 0.0;
      lookahead_point.pose.orientation = path_.poses[i + 1].pose.orientation;

      return true;
    }
  }

  // If we didn't find a point at lookahead distance, return the last point
  lookahead_point = path_.poses.back();
  return true;
}

double SimpleController::calculateDistance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2) const
{
  double dx = pose2.pose.position.x - pose1.pose.position.x;
  double dy = pose2.pose.position.y - pose1.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double SimpleController::calculateAngleDifference(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & target) const
{
  // Get current robot heading
  double robot_yaw = getYaw(current_pose);

  // Calculate angle to target
  double dx = target.pose.position.x - current_pose.pose.position.x;
  double dy = target.pose.position.y - current_pose.pose.position.y;
  double target_angle = std::atan2(dy, dx);

  // Return normalized angle difference
  return normalizeAngle(target_angle - robot_yaw);
}

double SimpleController::getYaw(const geometry_msgs::msg::PoseStamped & pose) const
{
  // Extract yaw from quaternion
  const auto & q = pose.pose.orientation;

  // Calculate yaw from quaternion
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double SimpleController::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

size_t SimpleController::findClosestPointIndex(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  if (path_.poses.empty()) {
    return 0;
  }

  size_t closest_index = current_path_index_;
  double min_distance = std::numeric_limits<double>::max();

  // Search from current index forward (don't go backward on path)
  for (size_t i = current_path_index_; i < path_.poses.size(); ++i) {
    double distance = calculateDistance(current_pose, path_.poses[i]);
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    } else if (distance > min_distance + lookahead_distance_) {
      // We've passed the closest point
      break;
    }
  }

  return closest_index;
}

}  // namespace ros2_astar_nav
