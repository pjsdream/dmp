#include <dmp/robot/planning_robot_model.h>
#include <dmp/robot/robot_model.h>
#include <dmp/robot/robot_joint.h>
#include <dmp/robot/robot_link.h>
#include <dmp/robot/planning_robot_joint.h>
#include <dmp/robot/planning_robot_link.h>
#include <dmp/utils/mesh_loader.h>
#include <dmp/shape/bounding_volume_factory.h>
#include <dmp/shape/aabb.h>
#include <dmp/shape/cube.h>

#include <iostream>

namespace dmp
{
PlanningRobotModel::PlanningRobotModel(const std::shared_ptr<RobotModel>& robot_model,
                                       const std::vector<std::string>& joint_names)
{
  std::unordered_set<std::string> joint_name_set(joint_names.cbegin(), joint_names.cend());

  links_.resize(1);

  const auto n = robot_model->numLinks();
  VectorEigen<Eigen::Affine3d> transforms(n, Eigen::Affine3d::Identity());
  std::vector<int> joint_index(n, 0);
  std::vector<int> link_index(n, 0);
  for (int i = 0; i < n; i++)
  {
    const auto& model_link = robot_model->getLink(i);

    if (i > 0)
    {
      const auto& model_joint = robot_model->getJoint(i);
      auto parent_index = robot_model->getParent(i);

      if (joint_name_set.find(model_joint.getName()) != joint_name_set.end())
      {
        PlanningRobotJoint joint(model_joint.getJointTypeAsString());
        joint.setOrigin(model_joint.getOrigin());
        joint.setAxis(model_joint.getAxis());
        joints_.push_back(joint);

        links_.emplace_back();

        joint_index[i] = static_cast<int>(joints_.size()) - 1;
        link_index[i] = static_cast<int>(links_.size()) - 1;
      }
      else
      {
        joint_index[i] = joint_index[parent_index];
        link_index[i] = link_index[parent_index];
        transforms[i] = transforms[parent_index] * model_joint.getJointTransform();
      }
    }

    // Generate collision shapes
    for (const auto& collision : model_link.getCollisions())
    {
      auto future_raw_mesh = MeshLoader::loadMesh(collision.filename);
      const auto& raw_mesh = future_raw_mesh.get();

      const auto num_vertices = raw_mesh.vertex_buffer.size() / 3;

      // Allocate vertex vector
      VectorEigen<Eigen::Vector3d> points(num_vertices);
      for (int j = 0; j < num_vertices; j++)
      {
        points[j] = collision.transform * Eigen::Vector3d(raw_mesh.vertex_buffer[3 * j],
                                                          raw_mesh.vertex_buffer[3 * j + 1],
                                                          raw_mesh.vertex_buffer[3 * j + 2]);
      }

      // Create bounding volume through factory function
      auto cube = std::make_shared<Cube>(BoundingVolumeFactory::newBoundingVolumeFromPoints(points)->as<AABB>());

      cube->setTransform(transforms[i] * cube->getTransform());

      links_[link_index[i]].addBoundingVolume(cube);
    }
  }
}

int PlanningRobotModel::numLinks() const noexcept
{
  return links_.size();
}

int PlanningRobotModel::numJoints() const noexcept
{
  return joints_.size();
}

const PlanningRobotLink& PlanningRobotModel::getLink(int index) const noexcept
{
  return links_[index];
}

const PlanningRobotJoint& PlanningRobotModel::getJoint(int index) const noexcept
{
  return joints_[index];
}
}
