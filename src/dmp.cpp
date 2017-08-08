#include <QApplication>
#include <dmp/rendering/renderer.h>
#include <dmp/rendering/scene/scene_object.h>
#include <dmp/rendering/scene/scene_node.h>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setSamples(4);
  format.setVersion(4, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  QSurfaceFormat::setDefaultFormat(format);

  auto renderer = std::make_unique<dmp::Renderer>();
  std::shared_ptr<dmp::SceneObject> object = renderer->createMeshObject("mesh");
  std::shared_ptr<dmp::SceneNode> node = renderer->createNode();
  node->attachObject(object);

  app.exec();
  return 0;
}
