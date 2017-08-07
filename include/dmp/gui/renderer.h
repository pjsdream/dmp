#ifndef DMP_RENDERER_H
#define DMP_RENDERER_H

#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLWidget>

#include <memory>

namespace dmp
{
class RendererImpl;
class Renderer
{
public:
  Renderer(QWidget* parent = nullptr);
  ~Renderer();

private:
  std::unique_ptr<RendererImpl> impl_;
};
}

#endif // DMP_RENDERER_H
