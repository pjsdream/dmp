#ifndef DMP_RENDERER_WINDOW_H
#define DMP_RENDERER_WINDOW_H

#include <renderer/gl_base.h>
#include <renderer/request/request.h>
#include <core/comm/subscriber.h>

#include <QOpenGLWindow>

#include <memory>
#include <unordered_map>

#include <Eigen/Dense>

class QMouseEvent;

namespace dmp
{
class Manager;
class SceneManager;
class SceneNode;
class SceneObject;
class ResourceManager;
class LightShader;
class RequestFrame;
class RequestMesh;
class RequestLight;
class RequestCustomTexture;
class RequestCustomMesh;
class LightManager;
class Camera;

class RendererWindow : public QOpenGLWindow
{
Q_OBJECT

public:
  RendererWindow();
  ~RendererWindow() override;

  RendererWindow(const RendererWindow& rhs) = delete;
  RendererWindow& operator=(const RendererWindow& rhs) = delete;

  RendererWindow(RendererWindow&& rhs) = delete;
  RendererWindow& operator=(RendererWindow&& rhs) = delete;

protected:
  void paintGL() override;
  void resizeGL(int w, int h) override;
  void initializeGL() override;

  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;

private:
  std::shared_ptr<GlFunctions> gl_;

  Subscriber request_subscriber_;

  std::unique_ptr<SceneManager> scene_manager_;
  std::unique_ptr<ResourceManager> resource_manager_;
  std::unique_ptr<LightManager> light_manager_;

  std::unique_ptr<LightShader> light_shader_;

  std::shared_ptr<Camera> camera_;

  int last_mouse_x_;
  int last_mouse_y_;
};
}

#endif // DMP_RENDERER_WINDOW_H
