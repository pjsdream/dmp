#include <dmp/planning/robot/planning_robot.h>
#include <dmp/robot/robot_model.h>
#include <dmp/robot/robot_link.h>
#include <dmp/robot/robot_joint.h>

namespace dmp
{
PlanningRobot::PlanningRobot(const RobotModel& robot_model)
{
  buildFromRobotModel(robot_model.getRoot());
}

void PlanningRobot::buildFromRobotModel(const std::shared_ptr<RobotLink>& model_link, int parent_link_id)
{
  // create link
  Link link;
  link.name = model_link->getName();
  link.parent = parent_link_id;

  for (const auto& model_visual : model_link->getVisuals())
  {
    Link::Visual visual;
    visual.filename = model_visual.filename;
    visual.has_color = model_visual.has_color;
    visual.color = model_visual.color;
    visual.transform = model_visual.transform;
    link.visuals.push_back(visual);
  }

  for (const auto& model_collision : model_link->getCollisions())
  {
  }

  links_.push_back(std::move(link));
  auto link_id = links_.size() - 1;

  // for root node, add a dummy joint (undefined joint)
  if (link_id == 0)
    joints_.resize(1);

  // traverse child joints
  for (const auto& model_child_joint : model_link->getChildJoints())
  {
    // create joint
    Joint joint;
    joint.name = model_child_joint->getName();
    joint.type = getJointType(model_child_joint->getJointTypeAsString());
    joint.origin = model_child_joint->getOrigin();
    joint.axis = model_child_joint->getAxis();
    joint.lower = model_child_joint->getLower();
    joint.upper = model_child_joint->getUpper();

    joints_.push_back(std::move(joint));

    buildFromRobotModel(model_child_joint->getChildLink(), link_id);
  }
}

PlanningRobot::Joint::Type PlanningRobot::getJointType(std::string type)
{
  if (type == "fixed")
    return Joint::Type::Fixed;
  else if (type == "continuous")
    return Joint::Type::Continuous;
  else if (type == "revolute")
    return Joint::Type::Revolute;
  else if (type == "prismatic")
    return Joint::Type::Prismatic;
  else
    return Joint::Type::Undefined;
}
}
