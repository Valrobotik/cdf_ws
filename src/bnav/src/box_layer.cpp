#include "bnav/box_layer.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bnav
{

void BoxLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("BoxLayer: costmap node is no longer available");
  }

  const auto topic = node->declare_parameter(
    getName() + ".map_topic", std::string("/box_map"));

  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic,
    std::bind(&BoxLayer::mapCallback, this, std::placeholders::_1),
    rclcpp::QoS(1).transient_local().reliable());

  RCLCPP_INFO(node->get_logger(), "BoxLayer: subscribing to %s", topic.c_str());
}

void BoxLayer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_ = msg;
}

void BoxLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!map_ || map_->info.width == 0 || map_->info.height == 0) {
    return;
  }

  const auto & info = map_->info;
  const double min_wx = info.origin.position.x;
  const double min_wy = info.origin.position.y;
  const double max_wx = min_wx + info.width * info.resolution;
  const double max_wy = min_wy + info.height * info.resolution;

  *min_x = std::min(*min_x, min_wx);
  *min_y = std::min(*min_y, min_wy);
  *max_x = std::max(*max_x, max_wx);
  *max_y = std::max(*max_y, max_wy);
}

void BoxLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!map_) {
    return;
  }

  const auto & info = map_->info;
  const double resolution = info.resolution;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;

  for (size_t j = 0; j < info.height; ++j) {
    for (size_t i = 0; i < info.width; ++i) {
      if (map_->data[j * info.width + i] < 50) {
        continue;
      }

      const double wx = origin_x + (static_cast<double>(i) + 0.5) * resolution;
      const double wy = origin_y + (static_cast<double>(j) + 0.5) * resolution;

      unsigned int mx = 0;
      unsigned int my = 0;
      if (!master_grid.worldToMap(wx, wy, mx, my)) {
        continue;
      }
      if (mx < static_cast<unsigned int>(min_i) ||
        mx >= static_cast<unsigned int>(max_i) ||
        my < static_cast<unsigned int>(min_j) ||
        my >= static_cast<unsigned int>(max_j))
      {
        continue;
      }

      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

void BoxLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_.reset();
}

}  // namespace bnav

PLUGINLIB_EXPORT_CLASS(bnav::BoxLayer, nav2_costmap_2d::Layer)
