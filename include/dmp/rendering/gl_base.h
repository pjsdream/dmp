#ifndef DMP_GL_BASE_H
#define DMP_GL_BASE_H

#include <memory>

#include <QOpenGLFunctions_4_3_Core>

namespace dmp
{
class GlBase
{
public:
  GlBase();
  explicit GlBase(std::shared_ptr<GlBase> base);
  virtual ~GlBase();

  GlBase(const GlBase& rhs) = delete;
  GlBase& operator=(const GlBase& rhs) = delete;

  GlBase(GlBase&& rhs) = delete;
  GlBase& operator=(GlBase&& rhs) = delete;

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
