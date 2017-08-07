#ifndef DMP_RENDERER_IMPL_H
#define DMP_RENDERER_IMPL_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_3_Core>

#include <dmp/gui/gl_base.h>

namespace dmp
{
class RendererImpl : public QOpenGLWidget, public GlBase
{
  Q_OBJECT

public:
  RendererImpl(QWidget* parent);
  ~RendererImpl();

protected:
  void paintGL() override;
  void resizeGL(int w, int h) override;
  void initializeGL() override;
};
}

#endif //DMP_RENDERER_IMPL_H
