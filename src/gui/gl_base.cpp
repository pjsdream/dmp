#include <dmp/gui/gl_base.h>

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