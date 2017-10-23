#include <dmp/rendering/renderer.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/json/json.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_light.h>
#include <dmp/planning/planner.h>
#include <dmp/planning/planning_option.h>
#include <dmp/robot/robot_model_loader.h>
#include <dmp/robot/robot_model.h>
#include <dmp/planning/environment/environment_loader.h>
#include <dmp/planning/environment/environment.h>
#include <dmp/planning/environment/interactable_object.h>
#include <dmp/planning/motion/motion_loader.h>
#include <dmp/shape/distance_query.h>
#include <dmp/shape/cube.h>
#include <dmp/shape/cylinder.h>
#include <dmp/shape/sphere.h>
#include <dmp/controlling/controller_option.h>
#include <dmp/controlling/controller.h>
#include <dmp/common.h>
#include <dmp/planning/objective/objective_reach_to_grip.h>
#include <dmp/planning/objective/objective_grip.h>
#include <dmp/planning/cost/cost_smoothness.h>
#include <dmp/planning/cost/cost_collision.h>
#include <dmp/comm/manager.h>

#include <QApplication>
#include <QWindow>

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

  // dmp communication manager
  auto manager = std::make_shared<dmp::Manager>();

  // robot model
  auto robot_model_loader = dmp::RobotModelLoader{};
  robot_model_loader.setSubstitutePackageDirectory(dmp::PROJECT_SOURCE_DIR + "/../../catkin_ws/src/fetch_ros");
  robot_model_loader.load(
      dmp::PROJECT_SOURCE_DIR + "/../../catkin_ws/src/fetch_ros/fetch_description/robots/fetch.urdf");
  auto robot_model = robot_model_loader.getRobotModel();

  // environment
  dmp::EnvironmentLoader environment_loader;
  auto environment = environment_loader.loadEnvironment(dmp::PROJECT_SOURCE_DIR + "/config/stacked_blocks.json");

  // motion
  dmp::MotionLoader motion_loader;
  auto motion = motion_loader.load(dmp::PROJECT_SOURCE_DIR + "/config/motion.json");

  // planner
  dmp::PlanningOption planning_option;
  planning_option.setRobotModel(robot_model);
  planning_option.setMotion(motion);
  planning_option.setEnvironment(environment);
  planning_option.setTrajectoryOptions(3.0, 5);
  planning_option.setTimestep(0.1);
  planning_option.setDiscretizations(30);

  auto planner = std::make_shared<dmp::Planner>(manager, planning_option);

  // task
  auto reach_to_grip =
      std::make_unique<dmp::ObjectiveReachToGrip>(std::dynamic_pointer_cast<dmp::InteractableObject>(environment->getObject(
          "red block")));
  auto grip =
      std::make_unique<dmp::ObjectiveGrip>(std::dynamic_pointer_cast<dmp::InteractableObject>(environment->getObject(
          "red block")));

  auto objective_publisher = manager->createPublisher<dmp::Objective>("objective");
  objective_publisher.publish(std::move(reach_to_grip));
  objective_publisher.publish(std::move(grip));

  // cost
  auto smoothness_cost = std::make_unique<dmp::CostSmoothness>(1e-2);
  auto collision_cost = std::make_unique<dmp::CostCollision>(1.);

  auto cost_publisher = manager->createPublisher<dmp::Cost>("cost");
  cost_publisher.publish(std::move(smoothness_cost));
  cost_publisher.publish(std::move(collision_cost));

  // controller
  dmp::ControllerOption controller_option;
  controller_option.setRobotModel(robot_model);
  controller_option.setMotion(motion);

  auto controller = std::make_shared<dmp::Controller>(manager, controller_option);

  // renderer
  auto renderer = std::make_shared<dmp::Renderer>(manager);

  auto light_publisher = manager->createPublisher<dmp::Request>("rendering");
  auto addLight = [&light_publisher](int index,
                                     auto type,
                                     auto position,
                                     auto ambient,
                                     auto diffuse,
                                     auto specular,
                                     auto attenuation)
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

    light_publisher.publish(std::move(light_req));
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

  // showing renderer
  renderer->resize(800, 600);
  renderer->show();

  // run threads
  planner->runThread();
  controller->runThread();

  app.exec();

  printf("exiting the program\n");
  planner->requestStop();
  controller->requestStop();
  planner->joinThread();
  controller->joinThread();

  renderer.reset();
  return 0;
}
