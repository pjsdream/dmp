#include <dmp/rendering/renderer.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/json/json.h>
#include <dmp/rendering/request/request.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_light.h>
#include <dmp/planning/planner.h>
#include <dmp/robot/robot_model_loader.h>

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
  robot_model_loader.substitutePackageDirectory("/Users/jaesungp/cpp_workspace/fetch_ros");
  robot_model_loader.load("/Users/jaesungp/cpp_workspace/fetch_ros/fetch_description/robots/fetch.urdf");
  auto robot_model = robot_model_loader.getRobotModel();

  auto renderer = std::make_shared<dmp::Renderer>();

  auto planner = std::make_shared<dmp::Planner>();
  planner->setRenderer(renderer);
  planner->setRobotModel(robot_model);

  auto frame = std::make_unique<dmp::RequestFrame>();
  frame->action = dmp::RequestFrame::Action::Set;
  frame->name = "bunny";
  frame->transform = Eigen::Affine3d::Identity();
  frame->transform.translate(Eigen::Vector3d(0.5, 0., 0.));

  auto mesh = std::make_unique<dmp::RequestMesh>();
  mesh->action = dmp::RequestMesh::Action::Attach;
  mesh->frame = "bunny";
  mesh->filename = "/Users/jaesungp/cpp_workspace/dmp/meshes/bunny.obj";

  auto light = dmp::Light{};
  light.type = dmp::Light::LightType::Directional;
  light.position = Eigen::Vector3f(0.f, 0.f, 1.f);
  light.ambient = Eigen::Vector3f(1.f, 1.f, 1.f);
  light.diffuse = Eigen::Vector3f(1.f, 1.f, 1.f);
  light.specular = Eigen::Vector3f(1.f, 1.f, 1.f);

  auto light_req = std::make_unique<dmp::RequestLight>();
  light_req->setLight(0, std::move(light));

  renderer->sendRequest(std::move(frame));
  renderer->sendRequest(std::move(mesh));
  renderer->sendRequest(std::move(light_req));

  app.exec();
  return 0;
}
