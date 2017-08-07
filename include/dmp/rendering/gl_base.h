#ifndef DMP_GL_BASE_H
#define DMP_GL_BASE_H

#include <QOpenGLFunctions_4_3_Core>
#include <memory>

namespace dmp
{
class GlBase
{
public:
  GlBase();
  GlBase(std::shared_ptr<GlBase> base);
  virtual ~GlBase();

  void initializeBaseGL(QOpenGLContext* context);

protected:
  std::shared_ptr<QOpenGLFunctions_4_3_Core> gl_;

private:
  std::shared_ptr<QOpenGLFunctions_4_3_Core> getGl()
  {
    return gl_;
  }
};
}

#endif //DMP_GL_BASE_H
