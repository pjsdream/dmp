#include <dmp/rendering/renderer_impl.h>

namespace dmp
{
RendererImpl::RendererImpl(QWidget* parent)
    : QOpenGLWidget(parent), GlBase()
{
}

RendererImpl::~RendererImpl() = default;

void RendererImpl::paintGL()
{
  gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void RendererImpl::resizeGL(int w, int h)
{
  gl_->glViewport(0, 0, w, h);
}

void RendererImpl::initializeGL()
{
  initializeBaseGL(context());

  gl_->glClearColor(0.8f, 0.8f, 0.8f, 0.f);
}
}