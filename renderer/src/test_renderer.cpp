#include <renderer/renderer_ostream.h>
#include <renderer/request/request_clear.h>
#include <renderer/request/request_mesh.h>
#include <renderer/request/request_light.h>
#include <renderer/request/request_frame.h>
#include <renderer/request/request_frame_attach.h>

#include <iostream>
#include <thread>

int main()
{
  dmp::RequestClear request_clear;

  dmp::RequestMesh request_mesh;
  request_mesh.name = "rect";
  request_mesh.filename = "/home/jaesungp/cpp_workspace/dmp/renderer/meshes/bunny.obj";

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

  dmp::RequestFrame request_frame;
  request_frame.action = dmp::RequestFrame::Action::Set;
  request_frame.name = "rect_frame";
  request_frame.transform = Eigen::Affine3d::Identity();

  dmp::RequestFrameAttach request_frame_attach;
  request_frame_attach.frame = "rect_frame";
  request_frame_attach.resource = "rect";

  std::cout << "sending request to renderer\n";

  using namespace std::chrono_literals;
  dmp::RendererOstream rout;
  rout << request_light0 << request_light1 << request_light2 << request_light3 << request_light4;
  rout.flush();

  //rout << request_clear << request_mesh << request_frame << request_frame_attach;
  rout.flush();

  std::cout << "sending request to renderer complete\n";

  rout << request_clear;

  request_mesh.name = "torso";
  request_mesh.filename = "/home/jaesungp/catkin_ws/src/fetch_ros/fetch_description/meshes/torso_lift_link.dae";

  request_frame.name = "torso_frame_up";
  request_frame.transform = Eigen::Affine3d::Identity();
  request_frame.transform.translate(Eigen::Vector3d(0, 0, 1));

  request_frame_attach.frame = "torso_frame_up";
  request_frame_attach.resource = "torso";

  rout << request_mesh << request_frame << request_frame_attach;
  rout.flush();

  return 0;
}
