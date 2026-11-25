#include "cdf_vacuum_gripper/vacuum_gripper_plugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/CollisionBitmask.hh>
#include <gz/sim/components/Collision.hh>

#include <algorithm>
#include <set>
#include <chrono>

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

void VacuumGripper::setCollisionCategories(
  gz::sim::EntityComponentManager & ecm)
{
  std::set<gz::sim::Entity> stoneCollisions;
  ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
    [&](const gz::sim::Entity & entity,
    const gz::sim::components::Model *,
    const gz::sim::components::Name * name)
    {
      if (name->Data().rfind(filter_, 0) != 0) {
        return true;
      }
      const auto link = gz::sim::Model(entity).LinkByName(ecm, "link");
      if (link != gz::sim::kNullEntity) {
        for (const auto & collision :
          ecm.ChildrenByComponents(link, gz::sim::components::Collision()))
        {
          stoneCollisions.insert(collision);
        }
      }
      return true;
    });

  ecm.Each<gz::sim::components::Collision>(
    [&](const gz::sim::Entity & entity,
    const gz::sim::components::Collision *)
    {
      const uint16_t category =
        stoneCollisions.count(entity) ? 2 : 1;
      if (!ecm.Component<gz::sim::components::CategoryBitmaskCmd>(entity)) {
        ecm.CreateComponent(
          entity, gz::sim::components::CategoryBitmaskCmd(category));
      } else {
        ecm.SetComponentData<gz::sim::components::CategoryBitmaskCmd>(
          entity, category);
      }
      return true;
    });
}

void VacuumGripper::setCarriedCollisionMask(
  gz::sim::EntityComponentManager & ecm, uint16_t mask)
{
  for (const auto & carried : carried_) {
    if (carried.link == gz::sim::kNullEntity) {
      continue;
    }
    for (const auto & collision :
      ecm.ChildrenByComponents(
        carried.link, gz::sim::components::Collision()))
    {
      if (!ecm.Component<gz::sim::components::CollideBitmaskCmd>(collision)) {
        ecm.CreateComponent(
          collision, gz::sim::components::CollideBitmaskCmd(mask));
      } else {
        ecm.SetComponentData<gz::sim::components::CollideBitmaskCmd>(
          collision, mask);
      }
    }
  }
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

  if (!categoriesSet_) {
    setCollisionCategories(ecm);
    categoriesSet_ = true;
  }

  // Carry the grasped stack kinematically. This keeps the robot's motion
  // independent from table/stack friction while preserving the stack shape.
  if (heldStone_ != gz::sim::kNullEntity) {
    const auto cupPose = gz::sim::worldPose(cupLink_, ecm);
    // Only collide with other boxes, never with the robot or the table.
    setCarriedCollisionMask(ecm, 2);
    for (const auto & carried : carried_) {
      ecm.SetComponentData<gz::sim::components::Pose>(
        carried.entity, cupPose * carried.offset);
    }
  }

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
        const auto cupPose = gz::sim::worldPose(cupLink_, ecm);
        const auto targetPose = gz::sim::worldPose(stone, ecm);

        carried_.clear();
        const auto targetLink =
          gz::sim::Model(stone).LinkByName(ecm, "link");
        carried_.push_back({stone, targetLink, cupPose.Inverse() * targetPose});

        // Include every stone directly above the target in the same stack,
        // so a bottom grasp carries the two boxes resting on it.
        ecm.Each<gz::sim::components::Model,
          gz::sim::components::Name,
          gz::sim::components::Pose>(
          [&](const gz::sim::Entity & entity,
          const gz::sim::components::Model *,
          const gz::sim::components::Name * name,
          const gz::sim::components::Pose * pose)
          {
            if (entity == stone || name->Data().rfind(filter_, 0) != 0) {
              return true;
            }
            const auto delta = pose->Data().Pos() - targetPose.Pos();
            if (std::abs(delta.X()) < 0.075 &&
                std::abs(delta.Y()) < 0.075 &&
                delta.Z() > -0.02 && delta.Z() < 0.5)
            {
              const auto link = gz::sim::Model(entity).LinkByName(ecm, "link");
              carried_.push_back({entity, link, cupPose.Inverse() * pose->Data()});
            }
            return true;
          });

        heldStone_ = stone;
        gzmsg << "VacuumGripper: carrying " << carried_.size()
              << " stone(s)" << std::endl;
        responseSuccess_ = true;
        responseMessage_ = "Stone attached";
      }
    }
  } else {
    if (heldStone_ == gz::sim::kNullEntity) {
      responseSuccess_ = true;
      responseMessage_ = "Nothing attached";
    } else {
      setCarriedCollisionMask(ecm, 0xFFFF);
      heldStone_ = gz::sim::kNullEntity;
      carried_.clear();
      responseSuccess_ = true;
      responseMessage_ = "Stone released";
    }
  }

  responseReady_ = true;
  cv_.notify_all();
}

}  // namespace cdf_vacuum_gripper
