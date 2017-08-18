#ifndef DMP_RENDERER_H
#define DMP_RENDERER_H

#include <dmp/rendering/gl_base.h>

#include <QOpenGLWidget>

#include <memory>

#include <Eigen/Dense>

class QMouseEvent;

namespace dmp
{
class SceneManager;
class SceneNode;
class SceneObject;
class ResourceManager;
class RequestManager;
class Request;
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

  void sendRequest(std::unique_ptr<Request> request);

protected:
  void paintGL() override;
  void resizeGL(int w, int h) override;
  void initializeGL() override;

  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}

#endif // DMP_RENDERER_H
