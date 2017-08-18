#include <dmp/rendering/renderer.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/json/json.h>
#include <dmp/rendering/request/request.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_light.h>
#include <dmp/planning/planner.h>
#include <dmp/robot/robot_model_loader.h>
#include <dmp/planning/environment/environment_loader.h>

#include <QApplication>

#include <thread>

int main(int argc, char** argv)
{
  setbuf(stdin, NULL);
  setbuf(stdout, NULL);

  QApplication app(argc, argv);

  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setSamples(4);
  format.setVersion(4, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  QSurfaceFormat::setDefaultFormat(format);

  auto robot_model_loader = dmp::RobotModelLoader{};
  robot_model_loader.setSubstitutePackageDirectory("/home/jaesungp/catkin_ws/src/fetch_ros");
  robot_model_loader.load("/home/jaesungp/catkin_ws/src/fetch_ros/fetch_description/robots/fetch.urdf");
  auto robot_model = robot_model_loader.getRobotModel();

  dmp::EnvironmentLoader environment_loader;
  auto environment = environment_loader.loadEnvironment("/home/jaesungp/cpp_workspace/dmp/config/environment.json");

  auto renderer = std::make_shared<dmp::Renderer>();

  auto planner = std::make_shared<dmp::Planner>();
  planner->setRenderer(renderer);
  planner->setRobotModel(robot_model);
  planner->setEnvironment(environment);

  auto addLight = [&](int index, auto position, auto ambient, auto diffuse, auto specular)
  {
    auto light = dmp::Light{};
    light.type = dmp::Light::LightType::Directional;
    light.position = position;
    light.ambient = ambient;
    light.diffuse = diffuse;
    light.specular = specular;

    auto light_req = std::make_unique<dmp::RequestLight>();
    light_req->setLight(index, std::move(light));

    renderer->sendRequest(std::move(light_req));
  };

  addLight(0,
           Eigen::Vector3f(0., 0., 1.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f));

  addLight(1,
           Eigen::Vector3f(-1., 0., 1.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f));

  addLight(2,
           Eigen::Vector3f(1., 0., 1.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f));

  addLight(3,
           Eigen::Vector3f(0., -1., 1.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f));

  addLight(4,
           Eigen::Vector3f(0., 1., 1.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f));

  app.exec();
  return 0;
}
