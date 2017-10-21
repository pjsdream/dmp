#include <dmp/planning/objective/objective_reach_to_grip.h>
#include <dmp/planning/environment/interactable_object.h>
#include <include/dmp/robot/robot_configuration.h>
#include <dmp/robot/planning_robot_model.h>

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
  // Robot link transform
  auto gripper_transform = configuration.getGripperTransformFromBase();

  // Object + grip position transform
  auto object_transform = object_->getTransform() * object_->getGripTransform();

  Eigen::Vector3d diff = gripper_transform.translation() - object_transform.translation();

  const auto cost = diff.squaredNorm();

  // Gradient
  Eigen::VectorXd gradient(configuration.getPositions().rows());

  auto robot_model = configuration.getRobotModel();

  for (int i = 0; i < robot_model->numJoints(); i++)
  {
    Eigen::Vector3d der = configuration.getGripperDerivativeTransform(i).block(0, 3, 3, 1);
    gradient(i) = 2. * der.dot(diff);
  }

  Eigen::VectorXd zeroes(configuration.getVelocities().rows());
  zeroes.setZero();

  return std::make_tuple(cost, gradient, zeroes);
}
}
