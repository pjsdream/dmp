#include <dmp/robot/robot_model_loader.h>
#include <dmp/robot/robot_model.h>

#include <tinyxml2/tinyxml2.h>

namespace dmp
{
void RobotModelLoader::substitutePackageDirectory(const std::string& directory)
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
            link.visual.geometry_filename = mesh->Attribute("filename");
        }

        if (auto material = visual->FirstChildElement("material"))
        {
          // TODO: material name

          if (auto color = material->FirstChildElement("color"))
          {
            sscanf(color->Attribute("rgba"),
                   "%lf%lf%lf%lf",
                   &link.visual.material_color[0],
                   &link.visual.material_color[1],
                   &link.visual.material_color[2],
                   &link.visual.material_color[3]);
          }
        }
      }

      if (auto collision = it->FirstChildElement("collision"))
      {
        readOrigin(link.collision.origin, collision);

        if (auto geometry = collision->FirstChildElement("geometry"))
        {
          if (auto mesh = geometry->FirstChildElement("mesh"))
            link.collision.geometry_filename = mesh->Attribute("filename");
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
}

std::shared_ptr<RobotModel> RobotModelLoader::getRobotModel()
{
  active_joints_.clear();
  for (auto link : links_)
    active_joints_.insert(link.first);

  traverse(root_name_, Eigen::Affine3d::Identity());

  return std::shared_ptr<RobotModel>();
}

std::shared_ptr<RobotModel> RobotModelLoader::getRobotModel(const std::vector<std::string>& active_joints)
{
  active_joints_ = std::unordered_set<std::string>(active_joints.begin(), active_joints.end());

  // TODO
  return std::shared_ptr<RobotModel>();
}

void RobotModelLoader::traverse(const std::string& link_name, const Eigen::Affine3d& transform)
{
  auto link = links_[link_name];

  for (auto joint_name : children_joints_[link_name])
  {
    auto joint = joints_[joint_name];
    auto child_link_name = joint.child;

    Eigen::Affine3d next_transform;
    if (active_joints_.find(joint_name) != active_joints_.cend())
    {
      next_transform = Eigen::Affine3d::Identity();
    }
    else
    {
      // TODO
      next_transform = transform;
    }
    traverse(child_link_name, next_transform);
  }
}
}
