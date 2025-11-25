// Copyright 2024 The Authors
// Licensed under the Apache License, Version 2.0

#ifndef ROS2_ASTAR_NAV__ASTAR_PLANNER_HPP_
#define ROS2_ASTAR_NAV__ASTAR_PLANNER_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_astar_nav
{

/**
 * @brief Structure representing a cell in the grid for A* algorithm
 */
struct AStarCell
{
  int x;           // X coordinate in grid
  int y;           // Y coordinate in grid
  double g_cost;   // Cost from start to this cell
  double h_cost;   // Heuristic cost from this cell to goal
  double f_cost;   // Total cost: f = g + h
  int parent_x;    // Parent cell X coordinate
  int parent_y;    // Parent cell Y coordinate

  AStarCell()
  : x(0), y(0), g_cost(0.0), h_cost(0.0), f_cost(0.0), parent_x(-1), parent_y(-1) {}

  AStarCell(int x_, int y_, double g_, double h_, int px, int py)
  : x(x_), y(y_), g_cost(g_), h_cost(h_), f_cost(g_ + h_), parent_x(px), parent_y(py) {}
};

/**
 * @brief Comparator for priority queue (min-heap based on f_cost)
 */
struct CompareCell
{
  bool operator()(const AStarCell & a, const AStarCell & b) const
  {
    return a.f_cost > b.f_cost;  // Min-heap: smaller f_cost has higher priority
  }
};

/**
 * @brief A* pathfinding algorithm implementation for 2D grid maps
 *
 * This class implements a classic A* pathfinding algorithm with support for:
 * - Euclidean distance heuristic
 * - 8-directional movement (diagonal allowed)
 * - Obstacle inflation for safety margin
 * - Configurable cost thresholds
 */
class AStarPlanner
{
public:
  /**
   * @brief Constructor for AStarPlanner
   */
  AStarPlanner();

  /**
   * @brief Destructor for AStarPlanner
   */
  ~AStarPlanner() = default;

  /**
   * @brief Set planner parameters
   * @param allow_diagonal Whether diagonal movement is allowed
   * @param heuristic_weight Weight for heuristic function
   * @param obstacle_threshold Occupancy value threshold for obstacles
   * @param inflation_radius Radius for obstacle inflation (meters)
   */
  void setParameters(
    bool allow_diagonal,
    double heuristic_weight,
    int obstacle_threshold,
    double inflation_radius);

  /**
   * @brief Set the map for planning
   * @param map Occupancy grid map
   */
  void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & map);

  /**
   * @brief Plan a path from start to goal
   * @param start Start pose
   * @param goal Goal pose
   * @param path Output path
   * @return true if path found, false otherwise
   */
  bool planPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path);

  /**
   * @brief Check if a position is valid (within bounds and not an obstacle)
   * @param x Grid X coordinate
   * @param y Grid Y coordinate
   * @return true if position is valid
   */
  bool isValid(int x, int y) const;

  /**
   * @brief Get the map resolution
   * @return Map resolution in meters per cell
   */
  double getMapResolution() const;

private:
  /**
   * @brief Calculate heuristic cost between two cells (Euclidean distance)
   * @param x1 Start X coordinate
   * @param y1 Start Y coordinate
   * @param x2 Goal X coordinate
   * @param y2 Goal Y coordinate
   * @return Heuristic cost
   */
  double heuristic(int x1, int y1, int x2, int y2) const;

  /**
   * @brief Get valid neighboring cells
   * @param cell Current cell
   * @param neighbors Output vector of neighbor cells
   */
  void getNeighbors(const AStarCell & cell, std::vector<std::pair<int, int>> & neighbors) const;

  /**
   * @brief Reconstruct path from goal to start using parent chain
   * @param goal_x Goal X coordinate
   * @param goal_y Goal Y coordinate
   * @param came_from Map of cell to parent cell
   * @param start Start pose for frame_id
   * @param goal Goal pose for final orientation
   * @param path Output path
   */
  void reconstructPath(
    int goal_x, int goal_y,
    const std::unordered_map<int, AStarCell> & came_from,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path);

  /**
   * @brief Convert grid coordinates to index
   * @param x Grid X coordinate
   * @param y Grid Y coordinate
   * @return Linear index in map data array
   */
  int gridToIndex(int x, int y) const;

  /**
   * @brief Convert world coordinates to grid coordinates
   * @param wx World X coordinate
   * @param wy World Y coordinate
   * @param gx Output grid X coordinate
   * @param gy Output grid Y coordinate
   * @return true if conversion successful
   */
  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;

  /**
   * @brief Convert grid coordinates to world coordinates
   * @param gx Grid X coordinate
   * @param gy Grid Y coordinate
   * @param wx Output world X coordinate
   * @param wy Output world Y coordinate
   */
  void gridToWorld(int gx, int gy, double & wx, double & wy) const;

  /**
   * @brief Check if a cell is an obstacle
   * @param x Grid X coordinate
   * @param y Grid Y coordinate
   * @return true if cell is an obstacle
   */
  bool isObstacle(int x, int y) const;

  /**
   * @brief Apply inflation to map around obstacles
   */
  void inflateObstacles();

  // Map data
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::vector<int8_t> inflated_map_;
  std::mutex map_mutex_;

  // Map dimensions
  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;

  // Planner parameters
  bool allow_diagonal_;
  double heuristic_weight_;
  int obstacle_threshold_;
  double inflation_radius_;

  // Direction vectors for 8-directional movement
  const std::vector<std::pair<int, int>> directions_8_ = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},    // Cardinal directions
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}   // Diagonal directions
  };

  // Direction vectors for 4-directional movement
  const std::vector<std::pair<int, int>> directions_4_ = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}     // Cardinal directions only
  };

  // Cost for diagonal movement (sqrt(2))
  static constexpr double DIAGONAL_COST = 1.41421356237;
  static constexpr double CARDINAL_COST = 1.0;
};

}  // namespace ros2_astar_nav

#endif  // ROS2_ASTAR_NAV__ASTAR_PLANNER_HPP_
