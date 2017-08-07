#ifndef DMP_RENDERER_H
#define DMP_RENDERER_H

#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLWidget>

#include <memory>

namespace dmp
{
class RendererImpl;
class SceneManager;
class Renderer
{
public:
  Renderer(QWidget* parent = nullptr);
  ~Renderer();

private:
  std::unique_ptr<RendererImpl> impl_;
  std::unique_ptr<SceneManager> scene_manager_;
};
}

#endif // DMP_RENDERER_H
