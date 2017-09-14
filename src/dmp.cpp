#include <dmp/rendering/renderer.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/json/json.h>
#include <dmp/rendering/request/request.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_light.h>
#include <dmp/planning/planner.h>
#include <dmp/planning/planning_option.h>
#include <dmp/robot/robot_model_loader.h>
#include <dmp/planning/environment/environment_loader.h>
#include <dmp/planning/motion/motion_loader.h>

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
  robot_model_loader.setSubstitutePackageDirectory("/playpen/jaesungp/catkin_ws/src/fetch_ros");
  robot_model_loader.load("/playpen/jaesungp/catkin_ws/src/fetch_ros/fetch_description/robots/fetch.urdf");
  auto robot_model = robot_model_loader.getRobotModel();

  dmp::EnvironmentLoader environment_loader;
  auto environment = environment_loader.loadEnvironment("/playpen/jaesungp/cpp_workspace/dmp/config/environment.json");

  dmp::MotionLoader motion_loader;
  auto motion = motion_loader.load("/playpen/jaesungp/cpp_workspace/dmp/config/motion.json");

  auto renderer = std::make_shared<dmp::Renderer>();

  dmp::PlanningOption option;
  option.setRenderer(renderer);
  option.setRobotModel(robot_model);
  option.setMotion(motion);
  option.setEnvironment(environment);

  auto planner = std::make_shared<dmp::Planner>(option);
  planner->plan();

  auto addLight = [&](int index, auto type, auto position, auto ambient, auto diffuse, auto specular, auto attenuation)
  {
    auto light = dmp::Light{};
    light.type = type;
    light.position = position;
    light.ambient = ambient;
    light.diffuse = diffuse;
    light.specular = specular;
    light.attenuation = attenuation;

    auto light_req = std::make_unique<dmp::RequestLight>();
    light_req->setLight(index, std::move(light));

    renderer->sendRequest(std::move(light_req));
  };

  addLight(0,
           dmp::Light::LightType::Directional,
           Eigen::Vector3f(0., 0., 1.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(1.f, 0.07f, 0.017f));

  addLight(1,
           dmp::Light::LightType::Point,
           Eigen::Vector3f(2., -2., 2.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.25f, 0.25f, 0.25f),
           Eigen::Vector3f(0.15f, 0.15f, 0.15f),
           Eigen::Vector3f(1.f, 0.07f, 0.017f));

  addLight(2,
           dmp::Light::LightType::Point,
           Eigen::Vector3f(2., 2., 2.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.25f, 0.25f, 0.25f),
           Eigen::Vector3f(0.15f, 0.15f, 0.15f),
           Eigen::Vector3f(1.f, 0.07f, 0.017f));

  addLight(3,
           dmp::Light::LightType::Point,
           Eigen::Vector3f(0., -2., 2.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.25f, 0.25f, 0.25f),
           Eigen::Vector3f(0.15f, 0.15f, 0.15f),
           Eigen::Vector3f(1.f, 0.07f, 0.017f));

  addLight(4,
           dmp::Light::LightType::Point,
           Eigen::Vector3f(0., 2., 2.),
           Eigen::Vector3f(0.1f, 0.1f, 0.1f),
           Eigen::Vector3f(0.25f, 0.25f, 0.25f),
           Eigen::Vector3f(0.15f, 0.15f, 0.15f),
           Eigen::Vector3f(1.f, 0.07f, 0.017f));

  app.exec();
  return 0;
}
