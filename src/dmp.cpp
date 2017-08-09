#include <QApplication>
#include <dmp/rendering/renderer.h>
#include <dmp/rendering/scene/scene_object.h>
#include <dmp/rendering/scene/scene_node.h>

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

  auto renderer = std::make_unique<dmp::Renderer>();

  app.exec();
  return 0;
}
