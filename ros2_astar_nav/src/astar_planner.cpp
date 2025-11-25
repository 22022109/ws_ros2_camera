// Copyright 2024 The Authors
// Licensed under the Apache License, Version 2.0

#include "ros2_astar_nav/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ros2_astar_nav
{

AStarPlanner::AStarPlanner()
: width_(0),
  height_(0),
  resolution_(0.0),
  origin_x_(0.0),
  origin_y_(0.0),
  allow_diagonal_(true),
  heuristic_weight_(1.0),
  obstacle_threshold_(90),
  inflation_radius_(0.3)
{
}

void AStarPlanner::setParameters(
  bool allow_diagonal,
  double heuristic_weight,
  int obstacle_threshold,
  double inflation_radius)
{
  allow_diagonal_ = allow_diagonal;
  heuristic_weight_ = heuristic_weight;
  obstacle_threshold_ = obstacle_threshold;
  inflation_radius_ = inflation_radius;
}

void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  map_ = map;
  width_ = static_cast<int>(map->info.width);
  height_ = static_cast<int>(map->info.height);
  resolution_ = map->info.resolution;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;

  // Apply inflation to obstacles
  inflateObstacles();
}

bool AStarPlanner::planPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (!map_) {
    return false;
  }

  // Convert world coordinates to grid coordinates
  int start_x, start_y, goal_x, goal_y;
  if (!worldToGrid(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    return false;
  }
  if (!worldToGrid(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    return false;
  }

  // Check if start and goal are valid
  if (!isValid(start_x, start_y) || !isValid(goal_x, goal_y)) {
    return false;
  }

  // Check if start or goal is on an obstacle
  if (isObstacle(start_x, start_y) || isObstacle(goal_x, goal_y)) {
    return false;
  }

  // A* algorithm implementation
  std::priority_queue<AStarCell, std::vector<AStarCell>, CompareCell> open_set;
  std::unordered_map<int, AStarCell> came_from;
  std::unordered_set<int> closed_set;

  // Add start cell to open set
  double h_start = heuristic(start_x, start_y, goal_x, goal_y);
  AStarCell start_cell(start_x, start_y, 0.0, h_start, -1, -1);
  open_set.push(start_cell);
  came_from[gridToIndex(start_x, start_y)] = start_cell;

  while (!open_set.empty()) {
    // Get cell with lowest f_cost
    AStarCell current = open_set.top();
    open_set.pop();

    int current_idx = gridToIndex(current.x, current.y);

    // Check if we reached the goal
    if (current.x == goal_x && current.y == goal_y) {
      reconstructPath(goal_x, goal_y, came_from, start, goal, path);
      return true;
    }

    // Skip if already processed
    if (closed_set.find(current_idx) != closed_set.end()) {
      continue;
    }
    closed_set.insert(current_idx);

    // Get neighbors
    std::vector<std::pair<int, int>> neighbors;
    getNeighbors(current, neighbors);

    for (const auto & neighbor : neighbors) {
      int nx = neighbor.first;
      int ny = neighbor.second;
      int neighbor_idx = gridToIndex(nx, ny);

      // Skip if already in closed set
      if (closed_set.find(neighbor_idx) != closed_set.end()) {
        continue;
      }

      // Calculate movement cost
      double movement_cost;
      if (std::abs(nx - current.x) + std::abs(ny - current.y) == 2) {
        // Diagonal movement
        movement_cost = DIAGONAL_COST;
      } else {
        // Cardinal movement
        movement_cost = CARDINAL_COST;
      }

      double new_g_cost = current.g_cost + movement_cost;

      // Check if this is a better path
      bool is_better = false;
      auto it = came_from.find(neighbor_idx);
      if (it == came_from.end()) {
        is_better = true;
      } else if (new_g_cost < it->second.g_cost) {
        is_better = true;
      }

      if (is_better) {
        double h_cost = heuristic(nx, ny, goal_x, goal_y);
        AStarCell neighbor_cell(nx, ny, new_g_cost, h_cost, current.x, current.y);
        came_from[neighbor_idx] = neighbor_cell;
        open_set.push(neighbor_cell);
      }
    }
  }

  // No path found
  return false;
}

bool AStarPlanner::isValid(int x, int y) const
{
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

double AStarPlanner::getMapResolution() const
{
  return resolution_;
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const
{
  // Euclidean distance heuristic
  double dx = static_cast<double>(x2 - x1);
  double dy = static_cast<double>(y2 - y1);
  return heuristic_weight_ * std::sqrt(dx * dx + dy * dy);
}

void AStarPlanner::getNeighbors(
  const AStarCell & cell,
  std::vector<std::pair<int, int>> & neighbors) const
{
  neighbors.clear();

  const auto & directions = allow_diagonal_ ? directions_8_ : directions_4_;

  for (const auto & dir : directions) {
    int nx = cell.x + dir.first;
    int ny = cell.y + dir.second;

    if (isValid(nx, ny) && !isObstacle(nx, ny)) {
      // For diagonal movement, check if we can move diagonally
      // (both adjacent cells must be free)
      if (allow_diagonal_ && dir.first != 0 && dir.second != 0) {
        // Check adjacent cells for diagonal movement
        if (isObstacle(cell.x + dir.first, cell.y) ||
          isObstacle(cell.x, cell.y + dir.second))
        {
          continue;  // Can't cut corners
        }
      }
      neighbors.emplace_back(nx, ny);
    }
  }
}

void AStarPlanner::reconstructPath(
  int goal_x, int goal_y,
  const std::unordered_map<int, AStarCell> & came_from,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & path)
{
  path.poses.clear();
  path.header.frame_id = start.header.frame_id;
  path.header.stamp = rclcpp::Clock().now();

  // Backtrack from goal to start
  std::vector<std::pair<int, int>> grid_path;
  int current_x = goal_x;
  int current_y = goal_y;

  while (current_x != -1 && current_y != -1) {
    grid_path.emplace_back(current_x, current_y);
    int idx = gridToIndex(current_x, current_y);
    auto it = came_from.find(idx);
    if (it != came_from.end()) {
      current_x = it->second.parent_x;
      current_y = it->second.parent_y;
    } else {
      break;
    }
  }

  // Reverse path (from start to goal)
  std::reverse(grid_path.begin(), grid_path.end());

  // Convert grid path to world coordinates
  for (size_t i = 0; i < grid_path.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;

    double wx, wy;
    gridToWorld(grid_path[i].first, grid_path[i].second, wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;

    // Calculate orientation from path direction
    if (i < grid_path.size() - 1) {
      double next_wx, next_wy;
      gridToWorld(grid_path[i + 1].first, grid_path[i + 1].second, next_wx, next_wy);
      double yaw = std::atan2(next_wy - wy, next_wx - wx);

      // Convert yaw to quaternion
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      // Use goal orientation for last pose
      pose.pose.orientation = goal.pose.orientation;
    }

    path.poses.push_back(pose);
  }
}

int AStarPlanner::gridToIndex(int x, int y) const
{
  return y * width_ + x;
}

bool AStarPlanner::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  if (resolution_ <= 0.0) {
    return false;
  }

  gx = static_cast<int>((wx - origin_x_) / resolution_);
  gy = static_cast<int>((wy - origin_y_) / resolution_);

  return isValid(gx, gy);
}

void AStarPlanner::gridToWorld(int gx, int gy, double & wx, double & wy) const
{
  wx = origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_;
  wy = origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_;
}

bool AStarPlanner::isObstacle(int x, int y) const
{
  if (!isValid(x, y)) {
    return true;
  }

  int idx = gridToIndex(x, y);

  // Use inflated map if available
  if (!inflated_map_.empty()) {
    return inflated_map_[idx] >= obstacle_threshold_ || inflated_map_[idx] < 0;
  }

  // Fall back to original map
  if (map_ && idx < static_cast<int>(map_->data.size())) {
    return map_->data[idx] >= obstacle_threshold_ || map_->data[idx] < 0;
  }

  return true;  // Consider unknown as obstacle
}

void AStarPlanner::inflateObstacles()
{
  if (!map_ || resolution_ <= 0.0) {
    return;
  }

  // Copy original map data
  inflated_map_.assign(map_->data.begin(), map_->data.end());

  // Calculate inflation radius in cells
  int inflate_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  // Inflate obstacles
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int idx = gridToIndex(x, y);

      // Check if this cell is an obstacle in original map
      if (map_->data[idx] >= obstacle_threshold_ || map_->data[idx] < 0) {
        // Inflate around this obstacle
        for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
          for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;

            if (!isValid(nx, ny)) {
              continue;
            }

            // Check if within circular inflation radius
            double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_;
            if (dist <= inflation_radius_) {
              int neighbor_idx = gridToIndex(nx, ny);
              // Mark as obstacle (or increase cost)
              if (inflated_map_[neighbor_idx] < obstacle_threshold_ &&
                inflated_map_[neighbor_idx] >= 0)
              {
                inflated_map_[neighbor_idx] = obstacle_threshold_;
              }
            }
          }
        }
      }
    }
  }
}

}  // namespace ros2_astar_nav
