#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

using namespace std::chrono_literals;

namespace bnav
{

class MissionRunner
{
public:
  explicit MissionRunner(nav2::LifecycleNode::SharedPtr node)
  : node_(std::move(node)),
    bt_xml_(node_->declare_parameter<std::string>("bt_xml", defaultTreePath())),
    groot_port_(node_->declare_parameter<int>("groot_port", 3003)),
    retry_delay_ms_(node_->declare_parameter<int>("retry_delay_ms", 2000)),
    plugin_libs_(node_->declare_parameter<std::vector<std::string>>(
      "plugin_libs", defaultPluginLibs()))
  {
    engine_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_libs_, node_);

    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", node_);
    blackboard_->set<std::chrono::milliseconds>("server_timeout", 20s);
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
    blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);

    executor_.add_node(node_->get_node_base_interface());
  }

  void run()
  {
    while (rclcpp::ok()) {
      runTreeOnce();

      if (!rclcpp::ok()) {
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(retry_delay_ms_));
    }
  }

private:
  void runTreeOnce()
  {
    try {
      auto tree = engine_->createTreeFromFile(bt_xml_, blackboard_);

      engine_->resetGrootMonitor();
      if (groot_port_ > 0) {
        engine_->addGrootMonitoring(&tree, static_cast<uint16_t>(groot_port_));
      }

      RCLCPP_INFO(node_->get_logger(), "Running behavior tree: %s", bt_xml_.c_str());
      const auto status = engine_->run(
        &tree,
        [this]() {executor_.spin_some();},
        []() {return !rclcpp::ok();});

      RCLCPP_INFO(
        node_->get_logger(), "Behavior tree finished: %s", statusName(status));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_->get_logger(), "Behavior tree failed (%s), retrying in %d ms",
        e.what(), retry_delay_ms_);
    }
  }

  static std::string defaultTreePath()
  {
    return (ament_index_cpp::get_package_share_path("bnav") / "bt" / "alternate.xml").string();
  }

  static std::vector<std::string> defaultPluginLibs()
  {
    return {
      "nav2_navigate_to_pose_action_bt_node",
      "random_pose_in_square_bt_node",
    };
  }

  static const char * statusName(nav2_behavior_tree::BtStatus status)
  {
    switch (status) {
      case nav2_behavior_tree::BtStatus::SUCCEEDED:
        return "SUCCEEDED";
      case nav2_behavior_tree::BtStatus::FAILED:
        return "FAILED";
      case nav2_behavior_tree::BtStatus::CANCELED:
        return "CANCELED";
    }
    return "UNKNOWN";
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::string bt_xml_;
  int groot_port_;
  int retry_delay_ms_;
  std::vector<std::string> plugin_libs_;
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> engine_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

}  // namespace bnav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<nav2::LifecycleNode>("alternate_goals_mission");
  node->autostart();

  try {
    bnav::MissionRunner runner(node);
    runner.run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Mission runner failed to start: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
