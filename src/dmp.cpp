#include <dmp/rendering/renderer.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/json/json.h>
#include <dmp/rendering/request/request.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/planning/planner.h>

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

  auto renderer = std::make_shared<dmp::Renderer>();

  auto planner = std::make_shared<dmp::Planner>();
  planner->setRenderer(renderer);

  auto frame = std::make_unique<dmp::RequestFrame>();
  frame->action = dmp::RequestFrame::Action::Set;
  frame->name = "bunny";
  frame->transform = Eigen::Affine3d::Identity();
  frame->transform.translate(Eigen::Vector3d(0.5, 0., 0.));

  auto mesh = std::make_unique<dmp::RequestMesh>();
  mesh->action = dmp::RequestMesh::Action::Attach;
  mesh->frame = "bunny";
  mesh->filename = "/Users/jaesungp/cpp_workspace/dmp/meshes/bunny.obj";

  renderer->sendRequest(std::move(frame));
  renderer->sendRequest(std::move(mesh));

  app.exec();
  return 0;
}
