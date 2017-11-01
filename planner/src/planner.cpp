#include <renderer/renderer_ostream.h>
#include <renderer/request/request_clear.h>
#include <renderer/request/request_light.h>
#include <planner/robot/robot_model_loader.h>

#include <QApplication>
#include <QWindow>

int main(int argc, char** argv)
{
  setbuf(stdin, NULL);
  setbuf(stdout, NULL);

  QApplication app(argc, argv);

  // TODO: set project source dir
  std::string PROJECT_SOURCE_DIR = "/home/jaesungp/cpp_workspace/dmp";

  // robot model
  auto robot_model_loader = dmp::RobotModelLoader{};
  robot_model_loader.setSubstitutePackageDirectory(PROJECT_SOURCE_DIR + "/../../catkin_ws/src/fetch_ros");
  robot_model_loader.load(
      PROJECT_SOURCE_DIR + "/../../catkin_ws/src/fetch_ros/fetch_description/robots/fetch.urdf");
  auto robot_model = robot_model_loader.getRobotModel();

  /*
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

   */

  // renderer
  dmp::RendererOstream rout;
  dmp::RequestClear request_clear;
  rout << request_clear;

  dmp::Light light;
  dmp::RequestLight request_light0;
  light.type = dmp::Light::LightType::Directional;
  light.position = Eigen::Vector3f(0., 0., 1.);
  light.ambient = Eigen::Vector3f(0.1f, 0.1f, 0.1f);
  light.diffuse = Eigen::Vector3f(0.25f, 0.25f, 0.25f);
  light.specular = Eigen::Vector3f(0.15f, 0.15f, 0.15f);
  light.attenuation = Eigen::Vector3f(1.f, 0.07f, 0.017f);
  request_light0.setLight(0, light);

  dmp::RequestLight request_light1;
  light.type = dmp::Light::LightType::Point;
  light.position = Eigen::Vector3f(2., -2., 2.);
  light.diffuse = Eigen::Vector3f(0.25f, 0.25f, 0.25f);
  light.specular = Eigen::Vector3f(0.15f, 0.15f, 0.15f);
  request_light1.setLight(1, light);

  dmp::RequestLight request_light2;
  light.position = Eigen::Vector3f(2., 2., 2.);
  request_light2.setLight(2, light);

  dmp::RequestLight request_light3;
  light.position = Eigen::Vector3f(0., -2., 2.);
  request_light3.setLight(3, light);

  dmp::RequestLight request_light4;
  light.position = Eigen::Vector3f(0., 2., 2.);
  request_light4.setLight(4, light);

  app.exec();
  return 0;
}
