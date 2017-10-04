#include <include/dmp/robot/robot_configuration.h>
#include <dmp/robot/robot_model.h>
#include <dmp/robot/robot_joint.h>
#include <dmp/robot/robot_link.h>
#include <dmp/planning/motion/motion.h>

namespace dmp
{
RobotConfiguration::RobotConfiguration(const std::shared_ptr<RobotModel>& robot_model,
                                       const std::shared_ptr<Motion>& motion)
    : robot_model_(robot_model), motion_(motion)
{
  transforms_.resize(robot_model_->numLinks());

  const auto& joint_names = motion->getBodyJoints();
  for (int i = 0; i < joint_names.size(); i++)
    joint_names_to_index_[joint_names[i]] = i;
}

const std::shared_ptr<Motion>& RobotConfiguration::getMotion() const noexcept
{
  return motion_;
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
      const auto& joint = robot_model_->getJoint(i);
      const auto parent = robot_model_->getParent(i);

      if (joint_names_to_index_.find(joint.getName()) == joint_names_to_index_.end())
        transforms_[i].matrix().noalias() = transforms_[parent].matrix() * joint.getJointTransform(0.).matrix();
      else
        transforms_[i].matrix().noalias() =
            transforms_[parent].matrix() * joint.getJointTransform(positions_[joint_names_to_index_[joint.getName()]]).matrix();
    }
  }
}

const Eigen::Affine3d& RobotConfiguration::getTransform(const std::string& link_name) const
{
  return transforms_[robot_model_->getLinkIndex(link_name)];
}
}
