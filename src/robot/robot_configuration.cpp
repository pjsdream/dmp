#include <include/dmp/robot/robot_configuration.h>
#include <dmp/robot/planning_robot_model.h>
#include <dmp/robot/planning_robot_joint.h>
#include <dmp/robot/planning_robot_link.h>
#include <dmp/planning/motion/motion.h>

namespace dmp
{
RobotConfiguration::RobotConfiguration(const std::shared_ptr<PlanningRobotModel>& robot_model)
    : robot_model_(robot_model)
{
  transforms_.resize(robot_model_->numLinks());
  gripper_transform_derivatives_.resize(robot_model_->numJoints());
}

void RobotConfiguration::setPositions(const Eigen::VectorXd& positions) noexcept
{
  positions_ = positions;
}

const Eigen::VectorXd& RobotConfiguration::getPositions() const noexcept
{
  return positions_;
}

void RobotConfiguration::setVelocities(const Eigen::VectorXd& velocities) noexcept
{
  velocities_ = velocities;
}

const Eigen::VectorXd& RobotConfiguration::getVelocities() const noexcept
{
  return velocities_;
}

void RobotConfiguration::forwardKinematics()
{
  const auto num_links = robot_model_->numLinks();
  for (int i = 0; i < num_links; i++)
  {
    const auto& link = robot_model_->getLink(i);

    Eigen::Affine3d transform;
    if (i == 0)
      transforms_[i] = Eigen::Affine3d::Identity();
    else
    {
      const auto& joint = robot_model_->getJoint(i - 1);
      const auto parent = robot_model_->getParentLinkIndex(i - 1);

      transforms_[i].matrix().noalias() =
          transforms_[parent].matrix() * joint.getJointTransform(positions_(i - 1)).matrix();
    }
  }
}

void RobotConfiguration::computeGripperTransformDerivative()
{
  const auto& gripper_indices = robot_model_->getLinkIndicesToGripper();
  const auto gripper_index = robot_model_->getGripperLinkIndex();

  // Reset all the matrices to zero
  gripper_transform_derivatives_.resize(robot_model_->numLinks(), Eigen::Matrix4d::Zero());

  for (const auto& link_index : gripper_indices)
  {
    // Skip the base link
    if (link_index == 0)
      continue;

    const auto joint_index = link_index - 1;
    const auto parent_link_index = robot_model_->getParentLinkIndex(joint_index);
    const auto& joint = robot_model_->getJoint(joint_index);

    gripper_transform_derivatives_[joint_index].matrix().noalias() =
        transforms_[parent_link_index] * joint.getJointDerivativeTransform(positions_(joint_index))
            * transforms_[link_index].matrix().inverse() * transforms_[gripper_index].matrix();
  }
}

const Eigen::Affine3d& RobotConfiguration::getTransform(int link_index) const
{
  return transforms_[link_index];
}

Eigen::Matrix4d RobotConfiguration::getGripperDerivativeTransform(int joint_index) const
{
  return gripper_transform_derivatives_[joint_index] * robot_model_->getGripperTransform().matrix();
}

Eigen::Affine3d RobotConfiguration::getGripperTransformFromBase() const
{
  return transforms_[robot_model_->getGripperLinkIndex()] * robot_model_->getGripperTransform();
}
}
