#include <planner/motion/motion_loader.h>
#include <planner/motion/motion.h>
#include <core/json/json_loader.h>
#include <core/json/json.h>

namespace dmp
{
std::shared_ptr<Motion> MotionLoader::load(const std::string& filename)
{
  JsonLoader json_loader;
  Json json{json_loader.loadJson(filename)};

  auto navigation = json["navigation"];
  auto& navigation_joints = navigation["joints"].toArray();
  for (auto joint : navigation_joints)
    navigation_joints_.push_back(joint.toString());

  auto body = json["body"];
  auto& body_joints = body["joints"].toArray();
  for (auto joint : body_joints)
    body_joints_.push_back(joint.toString());

  auto gripper = json["gripper"];
  auto& gripper_joints = gripper["joints"].toArray();
  for (auto joint : gripper_joints)
    gripper_joints_.push_back(joint.toString());

  auto gripping_position = gripper["gripping_position"];
  gripping_position_link_ = gripping_position["link"].toString();
  gripping_position_offset_ = Eigen::Vector3d(gripping_position["xyz"][0].toDouble(),
                                              gripping_position["xyz"][1].toDouble(),
                                              gripping_position["xyz"][2].toDouble());
  gripping_position_width_ = gripping_position["width"].toDouble();

  auto motion = std::make_shared<Motion>();
  motion->setNavigationJoints(navigation_joints_);
  motion->setBodyJoints(body_joints_);
  motion->setGripper(gripper_joints_, gripping_position_link_, gripping_position_offset_, gripping_position_width_);
  return motion;
}
}
