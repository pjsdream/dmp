#include <dmp/planning/objective/objective_reach_to_grip.h>
#include <dmp/planning/environment/interactable_object.h>
#include <include/dmp/robot/robot_configuration.h>
#include <dmp/planning/motion/motion.h>

#include <iostream>

namespace dmp
{
ObjectiveReachToGrip::ObjectiveReachToGrip(const std::shared_ptr<InteractableObject>& object)
    : object_(object)
{
}

std::tuple<double,
           Eigen::VectorXd,
           Eigen::VectorXd> ObjectiveReachToGrip::computeCost(const RobotConfiguration& configuration)
{
  const auto& motion = configuration.getMotion();
  const auto& gripper_link = motion->getGripperLink();
  const auto& gripper_xyz = motion->getGripperXyz();

  // Robot link transform
  auto link_transform = configuration.getTransform(gripper_link);
  link_transform.translate(gripper_xyz);

  // Object + grip position transform
  auto object_transform = object_->getTransform() * object_->getGripTransform();

  const auto cost = (link_transform.translation() - object_transform.translation()).squaredNorm();

  // TODO: Gradient
  Eigen::VectorXd zeroes(configuration.getPositions().rows());
  zeroes.setZero();

  return std::make_tuple(cost, zeroes, zeroes);
}
}
