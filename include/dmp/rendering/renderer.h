#ifndef DMP_RENDERER_H
#define DMP_RENDERER_H

#include <dmp/rendering/gl_base.h>

#include <QOpenGLWidget>

#include <memory>

#include <Eigen/Dense>

namespace dmp
{
class SceneManager;
class SceneNode;
class SceneObject;
class ResourceManager;
class RequestManager;
class LightShader;

class Renderer : public QOpenGLWidget
{
Q_OBJECT

public:
  explicit Renderer(QWidget* parent = nullptr);
  ~Renderer() override;

  Renderer(const Renderer& rhs) = delete;
  Renderer& operator=(const Renderer& rhs) = delete;

  Renderer(Renderer&& rhs) = delete;
  Renderer& operator=(Renderer&& rhs) = delete;

protected:
  void paintGL() override;
  void resizeGL(int w, int h) override;
  void initializeGL() override;

private:
  void traverseScene(const std::shared_ptr<SceneNode>& node, const Eigen::Affine3d& transform);

  std::shared_ptr<GlFunctions> gl_;

  std::unique_ptr<SceneManager> scene_manager_;
  std::unique_ptr<ResourceManager> resource_manager_;
  std::unique_ptr<RequestManager> request_manager_;

  std::unique_ptr<LightShader> light_shader_;
};
}

#endif // DMP_RENDERER_H
