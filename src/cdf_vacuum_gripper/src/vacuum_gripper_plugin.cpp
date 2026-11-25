#include "cdf_vacuum_gripper/vacuum_gripper_plugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <algorithm>
#include <chrono>
#include <set>

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
  modelEntity_ = entity;

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
  if (sdf->HasElement("max_lateral")) {
    maxLateral_ = sdf->Get<double>("max_lateral");
  }
  if (sdf->HasElement("clearance")) {
    clearance_ = sdf->Get<double>("clearance");
  }

  gz::sim::Model model(modelEntity_);
  cupLink_ = model.LinkByName(ecm, cupLinkName_);
  if (cupLink_ == gz::sim::kNullEntity) {
    gzerr << "VacuumGripper: cup link '" << cupLinkName_
          << "' not found on model '" << model.Name(ecm) << "'" << std::endl;
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
        << " on link " << cupLinkName_ << std::endl;
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
  const auto suctionAxis =
    cupPose.Rot().RotateVector(gz::math::Vector3d(0, 0, 1));

  std::set<gz::sim::Entity> heldLinks;
  ecm.Each<gz::sim::components::DetachableJoint>(
    [&heldLinks](const gz::sim::Entity &,
    const gz::sim::components::DetachableJoint * joint)
    {
      heldLinks.insert(joint->Data().childLink);
      return true;
    });

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
      auto link = model.LinkByName(ecm, "link");
      if (link == gz::sim::kNullEntity || heldLinks.count(link) != 0) {
        return true;
      }

      const auto delta = pose->Data().Pos() - cupPose.Pos();
      const double forward = delta.Dot(suctionAxis);

      // Project the box half-extents onto the suction axis. This is the
      // minimum center distance that keeps the box clear of the chassis.
      const auto boxRot = pose->Data().Rot();
      const double half_x = 0.055;
      const double half_y = 0.055;
      const double half_z = 0.16;
      const double extent =
        half_x * std::abs(suctionAxis.Dot(boxRot.RotateVector(gz::math::Vector3d(1, 0, 0)))) +
        half_y * std::abs(suctionAxis.Dot(boxRot.RotateVector(gz::math::Vector3d(0, 1, 0)))) +
        half_z * std::abs(suctionAxis.Dot(boxRot.RotateVector(gz::math::Vector3d(0, 0, 1))));
      if (forward < extent + clearance_) {
        return true;
      }

      const auto lateral = delta - suctionAxis * forward;
      if (lateral.Length() > maxLateral_) {
        return true;
      }

      const double distance = delta.Length();
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
        gz::sim::Model stoneModel(stone);
        const auto stoneLink = stoneModel.LinkByName(ecm, "link");
        if (stoneLink == gz::sim::kNullEntity) {
          responseSuccess_ = false;
          responseMessage_ = "Stone has no link";
        } else {
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
