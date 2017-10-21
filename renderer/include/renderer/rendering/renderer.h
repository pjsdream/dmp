#ifndef DMP_RENDERER_H
#define DMP_RENDERER_H

#include <dmp/rendering/gl_base.h>
#include <dmp/rendering/request/request.h>
#include <dmp/comm/subscriber.h>
#include <dmp/comm/node.h>

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

class Renderer : public QOpenGLWindow, public Node
{
Q_OBJECT

public:
  Renderer() = delete;
  Renderer(const std::shared_ptr<Manager>& manager, QWidget* parent = nullptr);
  ~Renderer() override;

  Renderer(const Renderer& rhs) = delete;
  Renderer& operator=(const Renderer& rhs) = delete;

  Renderer(Renderer&& rhs) = delete;
  Renderer& operator=(Renderer&& rhs) = delete;

protected:
  void paintGL() override;
  void resizeGL(int w, int h) override;
  void initializeGL() override;

  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;

private:
  void handleRequest(std::shared_ptr<Request> request);
  void handleRequestFrame(std::shared_ptr<RequestFrame> request);
  void handleRequestMesh(std::shared_ptr<RequestMesh> request);
  void handleRequestLight(std::shared_ptr<RequestLight> request);
  void handleRequestCustomTexture(std::shared_ptr<RequestCustomTexture> request);
  void handleRequestCustomMesh(std::shared_ptr<RequestCustomMesh> request);

  std::shared_ptr<GlFunctions> gl_;

  std::unique_ptr<SceneManager> scene_manager_;
  std::unique_ptr<ResourceManager> resource_manager_;
  std::unique_ptr<LightManager> light_manager_;

  Subscriber<Request> request_subscriber_;

  std::unique_ptr<LightShader> light_shader_;

  std::shared_ptr<Camera> camera_;

  int last_mouse_x_;
  int last_mouse_y_;
};
}

#endif // DMP_RENDERER_H
