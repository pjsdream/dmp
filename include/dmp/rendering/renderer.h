#ifndef DMP_RENDERER_H
#define DMP_RENDERER_H

#include <QOpenGLWidget>

#include <memory>

#include <Eigen/Dense>

#include <dmp/rendering/gl_base.h>
#include <dmp/rendering/scene/scene_manager.h>

namespace dmp
{
class SceneNode;
class SceneObject;

class Renderer : public QOpenGLWidget, public GlBase
{
  Q_OBJECT

public:
  explicit Renderer(QWidget* parent = nullptr);
  ~Renderer() override = default;

  Renderer(const Renderer& rhs) = delete;
  Renderer& operator=(const Renderer& rhs) = delete;

  Renderer(Renderer&& rhs) = delete;
  Renderer& operator=(Renderer&& rhs) = delete;

  std::shared_ptr<SceneNode> createNode();
  std::shared_ptr<SceneNode> createNode(const Eigen::Affine3d& transform);
  std::shared_ptr<SceneNode> createNode(const Eigen::Vector3d& translate);

  std::shared_ptr<SceneObject> createMeshObject(const std::string& filename);

protected:
  void paintGL() override;
  void resizeGL(int w, int h) override;
  void initializeGL() override;

private:
  std::unique_ptr<SceneManager> scene_manager_;
};
}

#endif // DMP_RENDERER_H
