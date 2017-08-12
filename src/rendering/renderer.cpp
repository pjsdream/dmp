#include <dmp/rendering/renderer.h>
#include <dmp/rendering/request/request_manager.h>
#include <dmp/rendering/request/request.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_light.h>
#include <dmp/rendering/resource/resource_manager.h>
#include <dmp/rendering/resource/resource_mesh.h>
#include <dmp/rendering/scene/scene_manager.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>
#include <dmp/rendering/shader/light_shader.h>
#include <dmp/rendering/camera/camera.h>
#include <dmp/rendering/light/light_manager.h>
#include <dmp/rendering/light/light.h>

#include <QTimer>
#include <QMouseEvent>

#include <iostream>

namespace dmp
{
class Renderer::Impl
{
public:
  Impl();
  ~Impl() = default;

  void paintGL();
  void resizeGL(int w, int h);
  void initializeGL(QOpenGLContext* context);

  void sendRequest(std::unique_ptr<Request> request);

  void mousePressEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);

private:
  void registerRequestHandlers();

  void handleRequest(std::unique_ptr<Request> request);
  void handleRequestFrame(std::unique_ptr<RequestFrame> request);
  void handleRequestMesh(std::unique_ptr<RequestMesh> request);
  void handleRequestLight(std::unique_ptr<RequestLight> request);

  std::shared_ptr<GlFunctions> gl_;

  std::unique_ptr<SceneManager> scene_manager_;
  std::unique_ptr<ResourceManager> resource_manager_;
  std::unique_ptr<RequestManager> request_manager_;
  std::unique_ptr<LightManager> light_manager_;

  std::unique_ptr<LightShader> light_shader_;

  std::shared_ptr<Camera> camera_;

  std::unordered_map<const std::type_info*, std::function<void(std::unique_ptr<Request>)>> request_handlers_;

  int last_mouse_x_;
  int last_mouse_y_;
};

Renderer::Renderer(QWidget* parent)
    : QOpenGLWidget(parent),
      impl_(std::make_unique<Impl>())
{
  // window management
  resize(800, 600);
  move(100, 100);
  show();

  QTimer* timer = new QTimer(this);
  timer->setInterval(16);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  timer->start();
}

Renderer::~Renderer() = default;

void Renderer::sendRequest(std::unique_ptr<Request> request)
{
  impl_->sendRequest(std::move(request));
}

void Renderer::paintGL()
{
  impl_->paintGL();
}

void Renderer::resizeGL(int w, int h)
{
  impl_->resizeGL(w, h);
}

void Renderer::initializeGL()
{
  impl_->initializeGL(context());
}

void Renderer::mousePressEvent(QMouseEvent* event)
{
  impl_->mousePressEvent(event);
  update();
}

void Renderer::mouseMoveEvent(QMouseEvent* event)
{
  impl_->mouseMoveEvent(event);
  update();
}

Renderer::Impl::Impl()
    : camera_(std::make_unique<Camera>()),
      scene_manager_(std::make_unique<SceneManager>()),
      request_manager_(std::make_unique<RequestManager>()),
      light_manager_(std::make_unique<LightManager>())
{
  registerRequestHandlers();
}

void Renderer::Impl::registerRequestHandlers()
{
  request_handlers_[&typeid(RequestFrame)] = [this](auto request)
  { handleRequestFrame(std::unique_ptr<RequestFrame>(dynamic_cast<RequestFrame*>(request.release()))); };
  request_handlers_[&typeid(RequestMesh)] = [this](auto request)
  { handleRequestMesh(std::unique_ptr<RequestMesh>(dynamic_cast<RequestMesh*>(request.release()))); };
  request_handlers_[&typeid(RequestLight)] = [this](auto request)
  { handleRequestLight(std::unique_ptr<RequestLight>(dynamic_cast<RequestLight*>(request.release()))); };
}

void Renderer::Impl::handleRequest(std::unique_ptr<Request> request)
{
  auto f = request_handlers_[&typeid(*request)];
  f(std::move(request));
}

void Renderer::Impl::handleRequestFrame(std::unique_ptr<RequestFrame> request)
{
  scene_manager_->setFrame(request->parent, request->name, request->transform);
}

void Renderer::Impl::handleRequestMesh(std::unique_ptr<RequestMesh> request)
{
  scene_manager_->attachResource(request->frame, resource_manager_->getMesh(request->filename));
}

void Renderer::Impl::handleRequestLight(std::unique_ptr<RequestLight> request)
{
  switch (request->getAction())
  {
    case RequestLight::Action::Set:
    {
      auto index = request->getIndex();
      auto light = request->getLight();

      light_manager_->setLight(index, light);
    }
    break;
    case RequestLight::Action::Delete:
    {
      auto index = request->getIndex();
      light_manager_->deleteLight(index);
    }
    break;
  }
}

void Renderer::Impl::sendRequest(std::unique_ptr<Request> request)
{
  request_manager_->addRequest(std::move(request));
}

void Renderer::Impl::paintGL()
{
  gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // update scene upon requests
  std::vector<std::unique_ptr<Request>> requests{request_manager_->pullRequests()};

  for (auto&& request : requests)
    handleRequest(std::move(request));

  // traverse scene
  auto nodes = scene_manager_->traverseNodes();

  // apply camera
  light_shader_->start();
  light_shader_->loadCamera(camera_);

  // apply lights
  auto lights = light_manager_->getLights();
  for (auto light : lights)
  {
    light_shader_->loadLight(light.first, light.second);
  }

  // rendering
  const Eigen::Vector3f default_color(0.8f, 0.8f, 0.8f);
  for (auto node : nodes)
  {
    light_shader_->loadModel(node->getTransform());
    auto resources = node->getAttachedResources();
    for (auto resource : resources)
    {
      auto mesh = std::dynamic_pointer_cast<ResourceMesh>(resource);

      // select color option for shader
      if (mesh->hasTexture())
        light_shader_->hasTexture();
      else if (mesh->hasColor())
        light_shader_->hasColor();
      else if (mesh->hasGlobalColor())
        light_shader_->hasGlobalColor(mesh->getGlobalColor());
      else
        light_shader_->hasGlobalColor(default_color);

      mesh->draw();
    }
  }
  light_shader_->end();
}

void Renderer::Impl::resizeGL(int w, int h)
{
  gl_->glViewport(0, 0, w, h);
}

void Renderer::Impl::initializeGL(QOpenGLContext* context)
{
  auto deleter = [](GlFunctions*)
  {};
  gl_.reset(context->versionFunctions<GlFunctions>(), deleter);

  gl_->glClearColor(0.8f, 0.8f, 0.8f, 0.f);
  gl_->glEnable(GL_DEPTH_TEST);

  resource_manager_ = std::make_unique<ResourceManager>(gl_);

  // shaders
  light_shader_ = std::make_unique<LightShader>(gl_);
}

void Renderer::Impl::mousePressEvent(QMouseEvent* event)
{
  last_mouse_x_ = event->x();
  last_mouse_y_ = event->y();
}

void Renderer::Impl::mouseMoveEvent(QMouseEvent* event)
{
  const int x = event->x();
  const int y = event->y();
  const int dx = x - last_mouse_x_;
  const int dy = y - last_mouse_y_;

  last_mouse_x_ = x;
  last_mouse_y_ = y;

  switch (event->buttons())
  {
    case Qt::LeftButton:camera_->rotatePixel(dx, dy);
      break;

    case Qt::RightButton:camera_->translatePixel(dx, dy);
      break;

    case Qt::LeftButton | Qt::RightButton:camera_->zoomPixel(dx, dy);
      break;
  }
}
}