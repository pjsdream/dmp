#include <dmp/planning/environment/environment_loader.h>
#include <dmp/planning/environment/environment.h>
#include <dmp/planning/environment/interactable_object.h>
#include <dmp/planning/environment/obstacle.h>
#include <dmp/json/json.h>
#include <dmp/json/json_loader.h>
#include <dmp/shape/cube.h>
#include <dmp/shape/sphere.h>
#include <dmp/shape/cylinder.h>

namespace dmp
{
std::shared_ptr<Environment> EnvironmentLoader::loadEnvironment(const std::string &filename)
{
  JsonLoader json_loader;
  Json json{json_loader.loadJson(filename)};

  auto environment = std::make_shared<Environment>();

  for (auto json_object : json.toArray())
  {
    std::shared_ptr<Object> object;
    auto name = json_object["name"].toString();
    auto type = json_object["type"].toString();
    auto geometry = json_object["geometry"];

    if (type == "static obstacle")
      object = std::make_shared<Obstacle>();
    else if (type == "interactable object")
      object = std::make_shared<InteractableObject>();

    std::shared_ptr<Shape> shape;
    auto shape_type = geometry["shape"].toString();
    Eigen::Vector4f rgba{geometry["rgba"][0].toDouble(), geometry["rgba"][1].toDouble(), geometry["rgba"][2].toDouble(),
                         geometry["rgba"][3].toDouble()};
    Eigen::Vector3d xyz{geometry["xyz"][0].toDouble(), geometry["xyz"][1].toDouble(), geometry["xyz"][2].toDouble()};
    Eigen::Vector3d rpy{geometry["rpy"][0].toDouble(), geometry["rpy"][1].toDouble(), geometry["rpy"][2].toDouble()};

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translate(xyz);
    transform.rotate(Eigen::AngleAxisd(rpy(0), Eigen::Vector3d(0, 0, 1)));
    transform.rotate(Eigen::AngleAxisd(rpy(1), Eigen::Vector3d(0, 1, 0)));
    transform.rotate(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d(1, 0, 0)));

    if (shape_type == "cube")
    {
      auto cube = std::make_shared<Cube>();
      Eigen::Vector3d
          size{geometry["size"][0].toDouble(), geometry["size"][1].toDouble(), geometry["size"][2].toDouble()};
      cube->setSize(size);
      cube->setTransform(transform);
      shape = cube;
    }
    else if (shape_type == "sphere")
    {
      auto sphere = std::make_shared<Sphere>();
      sphere->setRadius(geometry["radius"].toDouble());
      sphere->setPosition(transform.translation());
      shape = sphere;
    }
    else if (shape_type == "cylinder")
    {
      auto cylinder = std::make_shared<Cylinder>();
      cylinder->setDimension(geometry["radius"].toDouble(), geometry["height"].toDouble());
      cylinder->setTransform(transform);
      shape = cylinder;
    }

    if (shape != nullptr)
    {
      object->setName(name);
      object->setColor(rgba);
      object->setShape(shape);
      environment->addObject(object);
    }
  }

  return environment;
}
}
