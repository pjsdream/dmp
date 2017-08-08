#include <dmp/rendering/gl_base.h>

#include <QOpenGLFunctions_4_3_Core>

namespace dmp
{
GlBase::GlBase()
    : gl_()
{
}

GlBase::GlBase(std::shared_ptr<GlBase> base)
    : gl_(base->getGl())
{
}

GlBase::~GlBase() = default;

void GlBase::initializeBaseGL(QOpenGLContext* context)
{
  auto deleter = [](QOpenGLFunctions_4_3_Core*) {};
  gl_.reset(context->versionFunctions<QOpenGLFunctions_4_3_Core>(), deleter);
}
}