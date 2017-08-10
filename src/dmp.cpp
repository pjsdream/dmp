#include <dmp/rendering/renderer.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/json/json.h>
#include <dmp/rendering/request/request.h>
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

  {
    dmp::Json json{};
    json["action"] = "set frame";
    json["name"] = "bunny";
    for (int i=0; i<16; i++)
      json["transform"].add(dmp::Json(i%5 == 0 ? 1. : 0.));
    renderer->sendRequest(dmp::Request(std::move(json)));
  }

  {
    dmp::Json json{};
    json["action"] = "attach mesh";
    json["name"] = "bunny";
    json["filename"] = "/Users/jaesungp/cpp_workspace/dmp/meshes/bunny.obj";
    renderer->sendRequest(dmp::Request(std::move(json)));
  }

  app.exec();
  return 0;
}
