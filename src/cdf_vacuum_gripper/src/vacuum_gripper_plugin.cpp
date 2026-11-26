#include "cdf_vacuum_gripper/vacuum_gripper_plugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <chrono>
#include <cmath>

GZ_ADD_PLUGIN(
  cdf_vacuum_gripper::VacuumGripper,
  gz::sim::System,
  cdf_vacuum_gripper::VacuumGripper::ISystemConfigure,
  cdf_vacuum_gripper::VacuumGripper::ISystemPreUpdate)

namespace cdf_vacuum_gripper
{
namespace
{
constexpr auto kResponseTimeout = std::chrono::seconds(2);
}

VacuumGripper::~VacuumGripper()
{
  if (executor_) {
    executor_->cancel();
  }
  if (executorThread_.joinable()) {
    executorThread_.join();
  }
}

void VacuumGripper::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager &)
{
  if (sdf->HasElement("cup_link")) {
    cupLinkName_ = sdf->Get<std::string>("cup_link");
  }
  if (sdf->HasElement("service_name")) {
    serviceName_ = sdf->Get<std::string>("service_name");
  }
  if (sdf->HasElement("filter")) {
    filter_ = sdf->Get<std::string>("filter");
  }
  if (sdf->HasElement("max_distance")) {
    maxDistance_ = sdf->Get<double>("max_distance");
  }
  if (sdf->HasElement("lift")) {
    lift_ = sdf->Get<double>("lift");
  }

  gz::sim::Model model(entity);
  cupLink_ = model.LinkByName(ecm, cupLinkName_);
  if (cupLink_ == gz::sim::kNullEntity) {
    gzerr << "VacuumGripper: cup link '" << cupLinkName_
          << "' not found" << std::endl;
    return;
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = rclcpp::Node::make_shared("vacuum_gripper_" + cupLinkName_);
  node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
  service_ = node_->create_service<std_srvs::srv::SetBool>(
    serviceName_,
    std::bind(
      &VacuumGripper::handleRequest, this,
      std::placeholders::_1, std::placeholders::_2));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executorThread_ = std::thread([this]() {executor_->spin();});

  gzmsg << "VacuumGripper: " << serviceName_
        << " on " << cupLinkName_ << std::endl;
}

void VacuumGripper::handleRequest(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (requestPending_) {
    response->success = false;
    response->message = "Another vacuum request is pending";
    return;
  }

  requestPending_ = true;
  requestEnable_ = request->data;
  responseReady_ = false;
  cv_.notify_all();

  if (!cv_.wait_for(
      lock, kResponseTimeout, [this]() {return responseReady_;}))
  {
    requestPending_ = false;
    response->success = false;
    response->message = "Timed out waiting for the simulation";
    return;
  }

  response->success = responseSuccess_;
  response->message = responseMessage_;
  requestPending_ = false;
  responseReady_ = false;
}

gz::sim::Entity VacuumGripper::findClosestStone(
  gz::sim::EntityComponentManager & ecm)
{
  const auto cupPose = gz::sim::worldPose(cupLink_, ecm);

  gz::sim::Entity best = gz::sim::kNullEntity;
  double bestDistance = maxDistance_;

  ecm.Each<gz::sim::components::Model,
    gz::sim::components::Name,
    gz::sim::components::Pose>(
    [&](const gz::sim::Entity & entity,
    const gz::sim::components::Model *,
    const gz::sim::components::Name * name,
    const gz::sim::components::Pose * pose)
    {
      if (name->Data().rfind(filter_, 0) != 0) {
        return true;
      }

      gz::sim::Model model(entity);
      if (model.LinkByName(ecm, "link") == gz::sim::kNullEntity) {
        return true;
      }

      const double distance =
        (pose->Data().Pos() - cupPose.Pos()).Length();
      if (distance < bestDistance) {
        bestDistance = distance;
        best = entity;
      }
      return true;
    });

  return best;
}

void VacuumGripper::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager & ecm)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!requestPending_) {
    return;
  }

  if (requestEnable_) {
    if (heldStone_ != gz::sim::kNullEntity) {
      responseSuccess_ = true;
      responseMessage_ = "Already holding a stone";
    } else {
      const auto stone = findClosestStone(ecm);
      if (stone == gz::sim::kNullEntity) {
        responseSuccess_ = false;
        responseMessage_ = "No stone within reach";
      } else {
        const auto stoneLink =
          gz::sim::Model(stone).LinkByName(ecm, "link");
        if (stoneLink == gz::sim::kNullEntity) {
          responseSuccess_ = false;
          responseMessage_ = "Stone has no link";
        } else {
          // Lift the target and every box above it slightly off the table.
          // The fixed joint then carries the stack instead of dragging it.
          const auto targetPose = gz::sim::worldPose(stone, ecm);
          ecm.Each<gz::sim::components::Model,
            gz::sim::components::Name,
            gz::sim::components::Pose>(
            [&](const gz::sim::Entity & entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name * name,
            const gz::sim::components::Pose * pose)
            {
              if (name->Data().rfind(filter_, 0) != 0) {
                return true;
              }
              const auto delta = pose->Data().Pos() - targetPose.Pos();
              if (std::abs(delta.X()) < 0.075 &&
                  std::abs(delta.Y()) < 0.075 &&
                  delta.Z() > -0.02)
              {
                auto lifted = pose->Data();
                lifted.Pos().Z(lifted.Pos().Z() + lift_);
                ecm.SetComponentData<gz::sim::components::Pose>(
                  entity, lifted);
              }
              return true;
            });

          jointEntity_ = ecm.CreateEntity();
          ecm.CreateComponent(
            jointEntity_,
            gz::sim::components::DetachableJoint(
              {cupLink_, stoneLink, "fixed"}));
          ecm.CreateComponent(
            jointEntity_,
            gz::sim::components::Name("vacuum_joint"));
          heldStone_ = stone;
          responseSuccess_ = true;
          responseMessage_ = "Stone attached";
        }
      }
    }
  } else {
    if (heldStone_ == gz::sim::kNullEntity) {
      responseSuccess_ = true;
      responseMessage_ = "Nothing attached";
    } else {
      if (jointEntity_ != gz::sim::kNullEntity) {
        ecm.RequestRemoveEntity(jointEntity_);
      }
      jointEntity_ = gz::sim::kNullEntity;
      heldStone_ = gz::sim::kNullEntity;
      responseSuccess_ = true;
      responseMessage_ = "Stone released";
    }
  }

  responseReady_ = true;
  cv_.notify_all();
}

}  // namespace cdf_vacuum_gripper
