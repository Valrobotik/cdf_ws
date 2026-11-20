#ifndef BNAV__BOX_LAYER_HPP_
#define BNAV__BOX_LAYER_HPP_

#include <cstdint>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bnav
{

class BoxLayer : public nav2_costmap_2d::Layer
{
public:
  BoxLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::mutex mutex_;
};

}  // namespace bnav

#endif  // BNAV__BOX_LAYER_HPP_
