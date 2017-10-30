#include <renderer/renderer_ostream.h>
#include <renderer/request/request_clear.h>
#include <renderer/request/request_mesh.h>
#include <renderer/request/request_light.h>
#include <renderer/request/request_frame.h>

#include <iostream>
#include <thread>

int main()
{
  dmp::RequestClear request_clear;

  dmp::RequestMesh request_mesh;
  request_mesh.name = "rect";
  request_mesh.filename = "/home/jaesungp/cpp_workspace/dmp/renderer/meshes/rect.obj";

  dmp::Light light;
  light.type = dmp::Light::LightType::Directional;
  light.position = Eigen::Vector3f(0., 2., 2.);
  light.ambient = Eigen::Vector3f(0.1f, 0.1f, 0.1f);
  light.diffuse = Eigen::Vector3f(0.25f, 0.25f, 0.25f);
  light.specular = Eigen::Vector3f(0.15f, 0.15f, 0.15f);
  light.attenuation = Eigen::Vector3f(1.f, 0.07f, 0.017f);

  dmp::RequestLight request_light;
  request_light.setLight(0, std::move(light));

  dmp::RequestFrame request_frame;
  request_frame.action = dmp::RequestFrame::Action::Set;
  request_frame.name = "root";
  request_frame.parent = "rect_frame";
  request_frame.transform = Eigen::Affine3d::Identity();

  std::cout << "sending request to renderer\n";

  using namespace std::chrono_literals;
  dmp::RendererOstream rout;
  std::this_thread::sleep_for(100ms);
  rout << request_clear << request_mesh << request_light << request_frame;
  rout.flush();

  std::cout << "sending request to renderer complete\n";

  return 0;
}
