#ifndef CDF_VACUUM_GRIPPER__VACUUM_GRIPPER_PLUGIN_HPP_
#define CDF_VACUUM_GRIPPER__VACUUM_GRIPPER_PLUGIN_HPP_

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace cdf_vacuum_gripper
{

class VacuumGripper
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ~VacuumGripper() override;

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & event_mgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

private:
  void handleRequest(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  gz::sim::Entity findClosestStone(gz::sim::EntityComponentManager & ecm);

  gz::sim::Entity cupLink_{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity_{gz::sim::kNullEntity};
  gz::sim::Entity heldStone_{gz::sim::kNullEntity};

  std::string cupLinkName_;
  std::string serviceName_;
  std::string filter_{"stone_"};
  double maxDistance_{0.25};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executorThread_;

  std::mutex mutex_;
  std::condition_variable cv_;
  bool requestPending_{false};
  bool requestEnable_{false};
  bool responseReady_{false};
  bool responseSuccess_{false};
  std::string responseMessage_;
};

}  // namespace cdf_vacuum_gripper

#endif  // CDF_VACUUM_GRIPPER__VACUUM_GRIPPER_PLUGIN_HPP_
