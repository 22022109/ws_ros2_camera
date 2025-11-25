// Copyright 2024 The Authors
// Licensed under the Apache License, Version 2.0

#ifndef ROS2_ASTAR_NAV__NAVIGATION_NODE_HPP_
#define ROS2_ASTAR_NAV__NAVIGATION_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_astar_nav/astar_planner.hpp"
#include "ros2_astar_nav/simple_controller.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ros2_astar_nav
{

/**
 * @brief Main navigation node that integrates A* planner and Pure Pursuit controller
 *
 * This node provides:
 * - Action server for NavigateToPose requests
 * - Map and odometry subscriptions
 * - Velocity command publishing
 * - Path visualization
 * - TF2 integration for coordinate transformations
 */
class NavigationNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  /**
   * @brief Constructor for NavigationNode
   */
  explicit NavigationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for NavigationNode
   */
  ~NavigationNode() override;

private:
  /**
   * @brief Initialize ROS2 parameters
   */
  void initializeParameters();

  /**
   * @brief Initialize subscriptions, publishers, and action server
   */
  void initializeROS();

  /**
   * @brief Callback for map updates
   * @param msg Occupancy grid message
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Callback for odometry updates
   * @param msg Odometry message
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Handle goal request
   * @param uuid Goal UUID
   * @param goal Goal request
   * @return Goal response
   */
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  /**
   * @brief Handle cancel request
   * @param goal_handle Goal handle
   * @return Cancel response
   */
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);

  /**
   * @brief Handle accepted goal
   * @param goal_handle Goal handle
   */
  void handleAccepted(const std::shared_ptr<GoalHandleNavigate> goal_handle);

  /**
   * @brief Execute navigation to goal
   * @param goal_handle Goal handle
   */
  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle);

  /**
   * @brief Get current robot pose in global frame
   * @param pose Output pose
   * @return true if pose obtained successfully
   */
  bool getCurrentPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Publish zero velocity to stop the robot
   */
  void stopRobot();

  /**
   * @brief Transform pose to target frame
   * @param input_pose Input pose
   * @param output_pose Output pose
   * @param target_frame Target frame
   * @return true if transformation successful
   */
  bool transformPose(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & output_pose,
    const std::string & target_frame);

  // ROS2 interfaces
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  // TF2 components
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Planner and controller
  std::unique_ptr<AStarPlanner> planner_;
  std::unique_ptr<SimpleController> controller_;

  // Map and odometry data
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  std::mutex map_mutex_;
  std::mutex odom_mutex_;

  // Parameters
  bool allow_diagonal_;
  double heuristic_weight_;
  int obstacle_cost_threshold_;
  double inflation_radius_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double lookahead_distance_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  double control_frequency_;
  std::string robot_base_frame_;
  std::string global_frame_;

  // State flags
  bool map_received_;
  bool odom_received_;
  std::atomic<bool> navigation_active_;
};

}  // namespace ros2_astar_nav

#endif  // ROS2_ASTAR_NAV__NAVIGATION_NODE_HPP_
