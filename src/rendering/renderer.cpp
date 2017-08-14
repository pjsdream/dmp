#include <dmp/rendering/renderer.h>
#include <dmp/rendering/request/request_manager.h>
#include <dmp/rendering/request/request.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_light.h>
#include <dmp/rendering/request/request_custom_texture.h>
#include <dmp/rendering/request/request_custom_mesh.h>
#include <dmp/rendering/resource/resource_manager.h>
#include <dmp/rendering/resource/resource_mesh.h>
#include <dmp/rendering/scene/scene_manager.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>
#include <dmp/rendering/shader/light_shader.h>
#include <dmp/rendering/camera/camera.h>
#include <dmp/rendering/light/light_manager.h>
#include <dmp/rendering/light/light.h>
#include <dmp/utils/texture_loader.h>

#include <QTimer>
#include <QMouseEvent>

#include <iostream>
#include <include/dmp/utils/mesh_loader.h>

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
  void handleRequest(std::unique_ptr<Request> request);
  void handleRequestFrame(std::unique_ptr<RequestFrame> request);
  void handleRequestMesh(std::unique_ptr<RequestMesh> request);
  void handleRequestLight(std::unique_ptr<RequestLight> request);
  void handleRequestCustomTexture(std::unique_ptr<RequestCustomTexture> request);
  void handleRequestCustomMesh(std::unique_ptr<RequestCustomMesh> request);

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
  update();
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
}

void Renderer::Impl::handleRequest(std::unique_ptr<Request> request)
{
  Request* bare_pointer = request.release();
  if (RequestFrame* request_frame = dynamic_cast<RequestFrame*>(bare_pointer))
    handleRequestFrame(std::unique_ptr<RequestFrame>(request_frame));
  else if (RequestMesh* request_mesh = dynamic_cast<RequestMesh*>(bare_pointer))
    handleRequestMesh(std::unique_ptr<RequestMesh>(request_mesh));
  else if (RequestLight* request_light = dynamic_cast<RequestLight*>(bare_pointer))
    handleRequestLight(std::unique_ptr<RequestLight>(request_light));
  else if (RequestCustomTexture* request_custom_texture = dynamic_cast<RequestCustomTexture*>(bare_pointer))
    handleRequestCustomTexture(std::unique_ptr<RequestCustomTexture>(request_custom_texture));
  else if (RequestCustomMesh* request_custom_mesh = dynamic_cast<RequestCustomMesh*>(bare_pointer))
    handleRequestCustomMesh(std::unique_ptr<RequestCustomMesh>(request_custom_mesh));
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

void Renderer::Impl::handleRequestCustomTexture(std::unique_ptr<RequestCustomTexture> request)
{
  TextureLoaderRawTexture texture;
  texture.width = request->w;
  texture.height = request->h;
  texture.image = std::move(request->image);

  resource_manager_->createTexture(request->name, std::move(texture));
}

void Renderer::Impl::handleRequestCustomMesh(std::unique_ptr<RequestCustomMesh> request)
{
  MeshLoaderRawMesh raw_mesh;
  raw_mesh.vertex_buffer = std::move(request->vertex_buffer);
  raw_mesh.normal_buffer = std::move(request->normal_buffer);
  raw_mesh.texture_buffer = std::move(request->texture_buffer);
  raw_mesh.color_buffer = std::move(request->color_buffer);
  raw_mesh.face_buffer = std::move(request->face_buffer);

  auto mesh = resource_manager_->createMesh(request->name, std::move(raw_mesh));

  if (!request->texture_name.empty()) {
    auto texture = resource_manager_->getTexture(request->texture_name);
    mesh->setTexture(texture);
  }

  scene_manager_->attachResource(request->frame, mesh);
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
        light_shader_->hasTexture(mesh->getTexture());
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
  camera_->setAspect((double)w / h);
}

void Renderer::Impl::initializeGL(QOpenGLContext* context)
{
  auto deleter = [](GlFunctions*){};
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