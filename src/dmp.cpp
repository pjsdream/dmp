#include <QApplication>
#include <dmp/rendering/renderer.h>
#include <dmp/rendering/scene/scene_object.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/json/json.h>
#include <dmp/rendering/request/request.h>

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

  auto json = dmp::Json::createObject();
  json->add("action", dmp::Json::createString("add"));
  std::thread t([json, renderer]()
                {
                  for (int i=0; i<5; i++)
                  {
                    dmp::Request req;
                    req.setJson(json);
                    renderer->sendRequest(std::move(req));
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                  }
                });

  app.exec();
  t.join();
  return 0;
}
