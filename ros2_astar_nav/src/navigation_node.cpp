// Copyright 2024 The Authors
// Licensed under the Apache License, Version 2.0

#include "ros2_astar_nav/navigation_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

namespace ros2_astar_nav
{

NavigationNode::NavigationNode(const rclcpp::NodeOptions & options)
: Node("astar_navigation", options),
  map_received_(false),
  odom_received_(false),
  navigation_active_(false)
{
  // Initialize parameters
  initializeParameters();

  // Create planner and controller
  planner_ = std::make_unique<AStarPlanner>();
  controller_ = std::make_unique<SimpleController>();

  // Configure planner and controller with parameters
  planner_->setParameters(
    allow_diagonal_,
    heuristic_weight_,
    obstacle_cost_threshold_,
    inflation_radius_);

  controller_->setParameters(
    max_linear_velocity_,
    max_angular_velocity_,
    lookahead_distance_,
    goal_position_tolerance_,
    goal_orientation_tolerance_);

  // Initialize ROS interfaces
  initializeROS();

  RCLCPP_INFO(this->get_logger(), "A* Navigation Node initialized");
}

NavigationNode::~NavigationNode()
{
  navigation_active_ = false;
  stopRobot();
}

void NavigationNode::initializeParameters()
{
  // Declare and get planner parameters
  this->declare_parameter("allow_diagonal", true);
  this->declare_parameter("heuristic_weight", 1.0);
  this->declare_parameter("obstacle_cost_threshold", 90);
  this->declare_parameter("inflation_radius", 0.3);

  allow_diagonal_ = this->get_parameter("allow_diagonal").as_bool();
  heuristic_weight_ = this->get_parameter("heuristic_weight").as_double();
  obstacle_cost_threshold_ = this->get_parameter("obstacle_cost_threshold").as_int();
  inflation_radius_ = this->get_parameter("inflation_radius").as_double();

  // Declare and get controller parameters
  this->declare_parameter("max_linear_velocity", 0.5);
  this->declare_parameter("max_angular_velocity", 1.0);
  this->declare_parameter("lookahead_distance", 0.5);
  this->declare_parameter("goal_position_tolerance", 0.1);
  this->declare_parameter("goal_orientation_tolerance", 0.1);

  max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
  goal_orientation_tolerance_ = this->get_parameter("goal_orientation_tolerance").as_double();

  // Declare and get general parameters
  this->declare_parameter("control_frequency", 10.0);
  this->declare_parameter("robot_base_frame", "base_link");
  this->declare_parameter("global_frame", "map");

  control_frequency_ = this->get_parameter("control_frequency").as_double();
  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
  global_frame_ = this->get_parameter("global_frame").as_string();

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  allow_diagonal: %s", allow_diagonal_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  heuristic_weight: %.2f", heuristic_weight_);
  RCLCPP_INFO(this->get_logger(), "  obstacle_cost_threshold: %d", obstacle_cost_threshold_);
  RCLCPP_INFO(this->get_logger(), "  inflation_radius: %.2f", inflation_radius_);
  RCLCPP_INFO(this->get_logger(), "  max_linear_velocity: %.2f", max_linear_velocity_);
  RCLCPP_INFO(this->get_logger(), "  max_angular_velocity: %.2f", max_angular_velocity_);
  RCLCPP_INFO(this->get_logger(), "  lookahead_distance: %.2f", lookahead_distance_);
  RCLCPP_INFO(this->get_logger(), "  control_frequency: %.2f", control_frequency_);
}

void NavigationNode::initializeROS()
{
  // Create TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscriptions
  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&NavigationNode::mapCallback, this, std::placeholders::_1));

  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&NavigationNode::odomCallback, this, std::placeholders::_1));

  // Create publishers
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

  // Create action server
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this,
    "navigate_to_pose",
    std::bind(&NavigationNode::handleGoal, this, _1, _2),
    std::bind(&NavigationNode::handleCancel, this, _1),
    std::bind(&NavigationNode::handleAccepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "Action server 'navigate_to_pose' created");
}

void NavigationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  current_map_ = msg;
  planner_->setMap(msg);
  map_received_ = true;

  RCLCPP_INFO_ONCE(
    this->get_logger(),
    "Map received: %dx%d, resolution: %.3f",
    msg->info.width, msg->info.height, msg->info.resolution);
}

void NavigationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  current_odom_ = msg;
  odom_received_ = true;
}

rclcpp_action::GoalResponse NavigationNode::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(
    this->get_logger(),
    "Received goal request: (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // Check if we have the necessary data
  if (!map_received_) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: No map received yet");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!odom_received_) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: No odometry received yet");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Accept the goal
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationNode::handleCancel(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  navigation_active_ = false;
  stopRobot();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationNode::handleAccepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  // Execute in a new thread to avoid blocking
  std::thread{std::bind(&NavigationNode::execute, this, goal_handle)}.detach();
}

void NavigationNode::execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing navigation goal");

  navigation_active_ = true;
  auto start_time = this->now();

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  auto result = std::make_shared<NavigateToPose::Result>();

  // Get current pose
  geometry_msgs::msg::PoseStamped current_pose;
  if (!getCurrentPose(current_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get current pose");
    result->error_code = 1;  // Couldn't get current pose
    goal_handle->abort(result);
    navigation_active_ = false;
    return;
  }

  // Transform goal to global frame
  geometry_msgs::msg::PoseStamped goal_pose;
  if (!transformPose(goal->pose, goal_pose, global_frame_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform goal to global frame");
    result->error_code = 2;  // Transform error
    goal_handle->abort(result);
    navigation_active_ = false;
    return;
  }

  // Plan path
  nav_msgs::msg::Path path;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!planner_->planPath(current_pose, goal_pose, path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan path to goal");
      result->error_code = 3;  // Planning failed
      goal_handle->abort(result);
      navigation_active_ = false;
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Path planned with %zu poses", path.poses.size());

  // Publish path for visualization
  path_publisher_->publish(path);

  // Set path to controller
  controller_->setPath(path);

  // Control loop
  rclcpp::Rate rate(control_frequency_);
  geometry_msgs::msg::Twist cmd_vel;

  while (rclcpp::ok() && navigation_active_) {
    // Check for cancellation
    if (goal_handle->is_canceling()) {
      stopRobot();
      controller_->reset();
      result->error_code = 0;  // Cancelled
      goal_handle->canceled(result);
      navigation_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Goal cancelled");
      return;
    }

    // Get current pose
    if (!getCurrentPose(current_pose)) {
      RCLCPP_WARN(this->get_logger(), "Failed to get current pose, continuing...");
      rate.sleep();
      continue;
    }

    // Check if goal is reached
    if (controller_->isGoalReached(current_pose)) {
      stopRobot();
      controller_->reset();
      result->error_code = 0;  // Success
      goal_handle->succeed(result);
      navigation_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      return;
    }

    // Compute velocity command
    if (controller_->computeVelocity(current_pose, cmd_vel)) {
      cmd_vel_publisher_->publish(cmd_vel);
    } else {
      // Goal reached or error
      if (controller_->isGoalReached(current_pose)) {
        stopRobot();
        controller_->reset();
        result->error_code = 0;
        goal_handle->succeed(result);
        navigation_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return;
      }
    }

    // Publish feedback
    feedback->current_pose = current_pose;
    feedback->distance_remaining = controller_->getDistanceToGoal(current_pose);
    feedback->navigation_time = this->now() - start_time;
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  // If we exit the loop without reaching goal
  stopRobot();
  controller_->reset();
  result->error_code = 4;  // Navigation stopped
  goal_handle->abort(result);
  navigation_active_ = false;
}

bool NavigationNode::getCurrentPose(geometry_msgs::msg::PoseStamped & pose)
{
  // Try to get transform from base frame to global frame
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_,
      tf2::TimePointZero);

    pose.header.frame_id = global_frame_;
    pose.header.stamp = this->now();
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    return true;
  } catch (const tf2::TransformException & ex) {
    // Fall back to odometry if TF fails
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (current_odom_) {
      pose.header.frame_id = global_frame_;
      pose.header.stamp = this->now();
      pose.pose = current_odom_->pose.pose;
      return true;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Could not get current pose: %s", ex.what());
    return false;
  }
}

void NavigationNode::stopRobot()
{
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.linear.y = 0.0;
  stop_cmd.linear.z = 0.0;
  stop_cmd.angular.x = 0.0;
  stop_cmd.angular.y = 0.0;
  stop_cmd.angular.z = 0.0;

  cmd_vel_publisher_->publish(stop_cmd);
}

bool NavigationNode::transformPose(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & output_pose,
  const std::string & target_frame)
{
  if (input_pose.header.frame_id == target_frame) {
    output_pose = input_pose;
    return true;
  }

  try {
    output_pose = tf_buffer_->transform(input_pose, target_frame);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not transform pose from '%s' to '%s': %s",
      input_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());

    // If frame_id is empty, assume it's already in target frame
    if (input_pose.header.frame_id.empty()) {
      output_pose = input_pose;
      output_pose.header.frame_id = target_frame;
      return true;
    }

    return false;
  }
}

}  // namespace ros2_astar_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ros2_astar_nav::NavigationNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
