#include <planner/robot/robot_model_loader.h>
#include <planner/robot/tree_robot_model.h>
#include <planner/robot/tree_robot_link.h>
#include <planner/robot/tree_robot_joint.h>
#include <planner/robot/robot_model.h>

#include <tinyxml2/tinyxml2.h>

namespace dmp
{
void RobotModelLoader::setSubstitutePackageDirectory(const std::string& directory)
{
  package_directory_ = directory;
}

void RobotModelLoader::load(const std::string& filename)
{
  links_.clear();
  joints_.clear();
  num_parents_.clear();

  tinyxml2::XMLDocument doc;
  doc.LoadFile(filename.c_str());
  const auto* robot = doc.FirstChildElement();
  robot_name_ = robot->Attribute("name");

  auto readOrigin = [](auto origin, const auto element)
  {
    if (auto origin_element = element->FirstChildElement("origin"))
    {
      if (origin_element->FindAttribute("xyz"))
        sscanf(origin_element->Attribute("xyz"), "%lf%lf%lf", &origin[0], &origin[1], &origin[2]);
      else
        origin[0] = origin[1] = origin[2] = 0.;

      if (origin_element->FindAttribute("rpy"))
        sscanf(origin_element->Attribute("rpy"), "%lf%lf%lf", &origin[3], &origin[4], &origin[5]);
      else
        origin[3] = origin[4] = origin[5] = 0.;
    }
  };

  auto it = robot->FirstChildElement();
  while (it != 0)
  {
    if (std::string("link") == it->Value())
    {
      Link link;
      link.name = it->Attribute("name");

      if (auto inertial = it->FirstChildElement("inertial"))
      {
        readOrigin(link.inertial.origin, inertial);

        if (auto mass_element = inertial->FirstChildElement("mass"))
          sscanf(mass_element->Attribute("value"), "%lf", &link.inertial.mass);

        if (auto inertia_element = inertial->FirstChildElement("inertia"))
        {
          sscanf(inertia_element->Attribute("ixx"), "%lf", &link.inertial.inertia[0]);
          sscanf(inertia_element->Attribute("ixy"), "%lf", &link.inertial.inertia[1]);
          sscanf(inertia_element->Attribute("ixz"), "%lf", &link.inertial.inertia[2]);
          sscanf(inertia_element->Attribute("iyy"), "%lf", &link.inertial.inertia[3]);
          sscanf(inertia_element->Attribute("iyz"), "%lf", &link.inertial.inertia[4]);
          sscanf(inertia_element->Attribute("izz"), "%lf", &link.inertial.inertia[5]);
        }
      }

      if (auto visual = it->FirstChildElement("visual"))
      {
        readOrigin(link.visual.origin, visual);

        if (auto geometry = visual->FirstChildElement("geometry"))
        {
          if (auto mesh = geometry->FirstChildElement("mesh"))
            link.visual.geometry_filename = substitutePackageDirectory(mesh->Attribute("filename"));
        }

        if (auto material = visual->FirstChildElement("material"))
        {
          // TODO: material name

          if (auto color = material->FirstChildElement("color"))
          {
            link.visual.has_material = true;
            sscanf(color->Attribute("rgba"),
                   "%lf%lf%lf%lf",
                   &link.visual.material_color[0],
                   &link.visual.material_color[1],
                   &link.visual.material_color[2],
                   &link.visual.material_color[3]);
          }
          else
            link.visual.has_material = false;
        }
      }

      if (auto collision = it->FirstChildElement("collision"))
      {
        while (collision != 0)
        {
          // read link collision
          Link::Collision link_collision;

          readOrigin(link_collision.origin, collision);

          if (auto geometry = collision->FirstChildElement("geometry"))
          {
            if (auto mesh = geometry->FirstChildElement("mesh"))
              link_collision.geometry_filename = substitutePackageDirectory(mesh->Attribute("filename"));
          }

          // add link collision
          link.collisions.push_back(link_collision);

          // iterate to the next collision element
          collision = collision->NextSiblingElement("collision");
        }
      }

      links_[link.name] = std::move(link);
    }

    else if (std::string("joint") == it->Value())
    {
      Joint joint;
      joint.name = it->Attribute("name");
      joint.type = it->Attribute("type");

      readOrigin(joint.origin, it);

      joint.parent = it->FirstChildElement("parent")->Attribute("link");
      joint.child = it->FirstChildElement("child")->Attribute("link");

      if (auto axis = it->FirstChildElement("axis"))
        sscanf(axis->Attribute("xyz"), "%lf%lf%lf", &joint.axis[0], &joint.axis[1], &joint.axis[2]);
      else
      {
        joint.axis[0] = 1.;
        joint.axis[1] = 0.;
        joint.axis[2] = 0.;
      }

      if (auto limit = it->FirstChildElement("limit"))
      {
        sscanf(limit->Attribute("effort"), "%lf", &joint.limit.effort);
        sscanf(limit->Attribute("velocity"), "%lf", &joint.limit.velocity);

        if (limit->FindAttribute("lower"))
          sscanf(limit->Attribute("lower"), "%lf", &joint.limit.lower);
        else
          joint.limit.lower = 0.;

        if (limit->FindAttribute("upper"))
          sscanf(limit->Attribute("upper"), "%lf", &joint.limit.upper);
        else
          joint.limit.upper = 0.;
      }

      num_parents_[joint.child]++;
      children_joints_[joint.parent].insert(joint.name);
      joints_[joint.name] = std::move(joint);
    }

    it = it->NextSiblingElement();
  }

  // traverse from root link
  root_name_ = [this]()
  {
    for (auto link : links_)
    {
      auto link_name = link.first;
      if (num_parents_[link_name] == 0)
        return link_name;
    }

    // must not reach here
    return std::string("");
  }();

  setAllJointsActive();
}

void RobotModelLoader::setAllJointsActive()
{
  active_joints_.clear();
  for (auto joint : joints_)
    active_joints_.insert(joint.first);
}

void RobotModelLoader::setActiveJoints(const std::vector<std::string>& active_joints)
{
  active_joints_ = std::unordered_set<std::string>(active_joints.begin(), active_joints.end());
}

void RobotModelLoader::setJointValues(const std::unordered_map<std::string, double>& joint_values)
{
  joint_values_ = joint_values;
}

std::shared_ptr<RobotModel> RobotModelLoader::getRobotModel()
{
  auto root = std::make_shared<TreeRobotLink>();
  root->setName(root_name_);
  traverse(root, root_name_, Eigen::Affine3d::Identity());

  auto tree_robot_model = std::make_shared<TreeRobotModel>();
  tree_robot_model->setRoot(root);

  auto robot_model = std::make_shared<RobotModel>(*tree_robot_model);
  return robot_model;
}

void RobotModelLoader::traverse(const std::shared_ptr<TreeRobotLink>& node,
                                const std::string& link_name,
                                const Eigen::Affine3d& transform)
{
  auto link = links_[link_name];

  if (!link.visual.geometry_filename.empty())
  {
    if (link.visual.has_material)
      node->addVisualMesh(link.visual.geometry_filename,
                          transform * originToTransform(link.visual.origin),
                          Eigen::Vector4d(link.visual.material_color));
    else
      node->addVisualMesh(link.visual.geometry_filename, transform * originToTransform(link.visual.origin));
  }

  for (const auto& collision : link.collisions)
    node->addCollisionMesh(collision.geometry_filename, originToTransform(collision.origin));

  for (auto joint_name : children_joints_[link_name])
  {
    auto raw_joint = joints_[joint_name];
    auto child_link_name = raw_joint.child;
    auto child_link = links_[child_link_name];

    auto robot_joint = createTreeRobotJointFromRaw(raw_joint);

    std::shared_ptr<TreeRobotLink> next_node;
    Eigen::Affine3d next_transform;
    if (active_joints_.find(joint_name) != active_joints_.cend())
    {
      next_transform = Eigen::Affine3d::Identity();
      next_node = std::make_shared<TreeRobotLink>();
      next_node->setName(child_link.name);

      robot_joint->setParentLink(node);
      robot_joint->setChildLink(next_node);
      node->addChildJoint(robot_joint);
      next_node->setParentJoint(robot_joint);
    }
    else
    {
      next_node = node;
      next_transform = transform * robot_joint->getTransform(joint_values_[joint_name]);
    }
    traverse(next_node, child_link_name, next_transform);
  }
}

std::string RobotModelLoader::substitutePackageDirectory(const std::string& filename)
{
  if (filename.substr(0, 10) == "package://")
    return package_directory_ + "/" + filename.substr(10);
}

Eigen::Affine3d RobotModelLoader::originToTransform(const double origin[6])
{
  // xyz rpy
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(Eigen::Vector3d(origin[0], origin[1], origin[2]));
  transform.rotate(Eigen::AngleAxisd(origin[3], Eigen::Vector3d(0, 0, 1)));
  transform.rotate(Eigen::AngleAxisd(origin[4], Eigen::Vector3d(0, 1, 0)));
  transform.rotate(Eigen::AngleAxisd(origin[5], Eigen::Vector3d(1, 0, 0)));

  return transform;
}

std::shared_ptr<TreeRobotJoint> RobotModelLoader::createTreeRobotJointFromRaw(const Joint& raw_joint)
{
  auto joint = std::make_shared<TreeRobotJoint>(raw_joint.type);
  joint->setName(raw_joint.name);
  joint->setOrigin(originToTransform(raw_joint.origin));
  joint->setAxis(Eigen::Vector3d(raw_joint.axis));
  joint->setLimit(raw_joint.limit.lower, raw_joint.limit.upper);

  return joint;
}
}
