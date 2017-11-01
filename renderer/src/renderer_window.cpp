#include <renderer/renderer_window.h>
#include <renderer/resource/resource_manager.h>
#include <renderer/resource/resource_mesh.h>
#include <renderer/scene/scene_manager.h>
#include <renderer/scene/scene_node.h>
#include <renderer/scene/scene_edge.h>
#include <renderer/shader/light_shader.h>
#include <renderer/camera/camera.h>
#include <renderer/light/light_manager.h>
#include <renderer/light/light.h>
#include <renderer/request/request_clear.h>
#include <renderer/request/request_mesh.h>
#include <renderer/request/request_custom_mesh.h>
#include <renderer/request/request_custom_texture.h>
#include <renderer/request/request_light.h>
#include <renderer/request/request_frame.h>
#include <renderer/request/request_frame_attach.h>

#include <QMouseEvent>
#include <QTimer>

#include <iostream>
#include <include/renderer/resource/raw_mesh.h>
#include <include/renderer/resource/texture_loader.h>

namespace dmp
{
//
// RendererWindow::Impl
//
RendererWindow::RendererWindow()
    : request_subscriber_("renderer", "127.0.0.1"),
      camera_(std::make_unique<Camera>()),
      scene_manager_(std::make_unique<SceneManager>()),
      light_manager_(std::make_unique<LightManager>())
{
}

RendererWindow::~RendererWindow() = default;

bool RendererWindow::receiveRequest()
{
  if (request_subscriber_.eof())
    return false;

  Request::Type type;
  request_subscriber_ >> type;

  printf("type: %d\n", static_cast<int>(type));

  switch (type)
  {
    case Request::Type::Clear:
    {
      auto req = std::make_unique<RequestClear>();
      request_subscriber_ >> *req;
      std::cout << "Clear\n";
      requests_.push_back(std::move(req));
    }
      break;

    case Request::Type::Mesh:
    {
      auto req = std::make_unique<RequestMesh>();
      request_subscriber_ >> *req;

      std::cout << "Mesh name: " << req->name << ", filename: " << req->filename << "\n";
      requests_.push_back(std::move(req));
    }
      break;

    case Request::Type::CustomMesh:
    {
      auto req = std::make_unique<RequestCustomMesh>();
      request_subscriber_ >> *req;

      std::cout << "Custom Mesh name: " << req->name << ", # vertices: " << req->vertex_buffer.size() << "\n";
      requests_.push_back(std::move(req));
    }
      break;

    case Request::Type::CustomTexture:
    {
      auto req = std::make_unique<RequestCustomTexture>();
      request_subscriber_ >> *req;

      std::cout << "Custom Texture name: " << req->name << ", size = (" << req->w << ", " << req->h << ")\n";
      requests_.push_back(std::move(req));
    }
      break;

    case Request::Type::Light:
    {
      auto req = std::make_unique<RequestLight>();
      request_subscriber_ >> *req;

      std::cout << "Light action: " << static_cast<int>(req->action) << ", index: " << req->index << "\n";
      requests_.push_back(std::move(req));
    }
      break;

    case Request::Type::Frame:
    {
      auto req = std::make_unique<RequestFrame>();
      request_subscriber_ >> *req;

      std::cout << "Frame name: " << req->name << ", parent: " << req->parent << ", transform:\n"
                << req->transform.matrix() << "\n";
      requests_.push_back(std::move(req));
    }
      break;

    case Request::Type::FrameAttach:
    {
      auto req = std::make_unique<RequestFrameAttach>();
      request_subscriber_ >> *req;

      std::cout << "Frame Attach frame: " << req->frame << ", resource: " << req->resource << "\n";
      requests_.push_back(std::move(req));
    }
      break;
  }

  return true;
}

void RendererWindow::receiveRequests()
{
  requests_.clear();
  while (receiveRequest());
}

void RendererWindow::handleRequests()
{
  for (auto& request : requests_)
  {
    switch (request->type())
    {
      case Request::Type::Clear:
        handleRequest(std::unique_ptr<RequestClear>(static_cast<RequestClear*>(request.release())));
        break;

      case Request::Type::Mesh:
        handleRequest(std::unique_ptr<RequestMesh>(static_cast<RequestMesh*>(request.release())));
        break;

      case Request::Type::CustomMesh:
        handleRequest(std::unique_ptr<RequestCustomMesh>(static_cast<RequestCustomMesh*>(request.release())));
        break;

      case Request::Type::CustomTexture:
        handleRequest(std::unique_ptr<RequestCustomTexture>(static_cast<RequestCustomTexture*>(request.release())));
        break;

      case Request::Type::Frame:
        handleRequest(std::unique_ptr<RequestFrame>(static_cast<RequestFrame*>(request.release())));
        break;

      case Request::Type::FrameAttach:
        handleRequest(std::unique_ptr<RequestFrameAttach>(static_cast<RequestFrameAttach*>(request.release())));
        break;

      case Request::Type::Light:
        handleRequest(std::unique_ptr<RequestLight>(static_cast<RequestLight*>(request.release())));
        break;
    }
  }
}

void RendererWindow::handleRequest(std::unique_ptr<RequestClear> req)
{
  scene_manager_->clear();
}

void RendererWindow::handleRequest(std::unique_ptr<RequestMesh> req)
{
  resource_manager_->loadMesh(req->name, req->filename);
}

void RendererWindow::handleRequest(std::unique_ptr<RequestCustomMesh> req)
{
  RawMesh raw_mesh;
  raw_mesh.vertex_buffer = std::move(req->vertex_buffer);
  raw_mesh.normal_buffer = std::move(req->normal_buffer);
  raw_mesh.color_buffer = std::move(req->color_buffer);
  raw_mesh.face_buffer = std::move(req->face_buffer);
  raw_mesh.texture_buffer = std::move(req->texture_buffer);
  raw_mesh.texture_filename = std::move(req->texture_name);
  raw_mesh.global_color = req->getGlobalColor();
  raw_mesh.has_global_color = req->hasGlobalColor();

  // TODO: raw mesh material
  resource_manager_->createMesh(req->name, std::move(raw_mesh));
}

void RendererWindow::handleRequest(std::unique_ptr<RequestCustomTexture> req)
{
  TextureLoaderRawTexture raw_texture;
  raw_texture.width = req->w;
  raw_texture.height = req->h;
  raw_texture.image = std::move(req->image);

  resource_manager_->createTexture(req->name, std::move(raw_texture));
}

void RendererWindow::handleRequest(std::unique_ptr<RequestFrame> req)
{
  scene_manager_->setFrame(req->parent, req->name, req->transform);
}

void RendererWindow::handleRequest(std::unique_ptr<RequestFrameAttach> req)
{
  scene_manager_->attachResource(req->frame, resource_manager_->getMesh(req->resource));
}

void RendererWindow::handleRequest(std::unique_ptr<RequestLight> req)
{
  switch (req->action)
  {
    case RequestLight::Action::Nothing:
      break;

    case RequestLight::Action::Set:
    {
      Light light;
      switch (req->light_type)
      {
        case RequestLight::LightType::Directional:
          light.type = Light::LightType::Directional;
          break;
        case RequestLight::LightType::Point:
          light.type = Light::LightType::Point;
          break;
      }
      light.position = req->position;
      light.ambient = req->ambient;
      light.diffuse = req->diffuse;
      light.specular = req->specular;
      light.attenuation = req->attenuation;

      light_manager_->setLight(req->index, light);
    }
      break;

    case RequestLight::Action::Delete:
      light_manager_->deleteLight(req->index);
      break;
  }
}

void RendererWindow::paintGL()
{
  gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // TODO: Update scene upon requests
  receiveRequests();
  handleRequests();

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
  for (const auto& node : nodes)
  {
    light_shader_->loadModel(node->getTransform());
    const auto& resources = node->getAttachedResources();
    for (const auto& resource : resources)
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

void RendererWindow::resizeGL(int w, int h)
{
  gl_->glViewport(0, 0, w, h);
  camera_->setAspect((double) w / h);

  update();
}

void RendererWindow::initializeGL()
{
  auto deleter = [](GlFunctions*)
  {};
  gl_.reset(context()->versionFunctions<GlFunctions>(), deleter);

  gl_->glClearColor(0.8f, 0.8f, 0.8f, 0.f);
  gl_->glEnable(GL_DEPTH_TEST);
  gl_->glEnable(GL_MULTISAMPLE);

  resource_manager_ = std::make_unique<ResourceManager>(gl_);

  // shaders
  light_shader_ = std::make_unique<LightShader>(gl_);

  QTimer* timer = new QTimer(this);
  timer->setInterval(16);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  timer->start();
}

void RendererWindow::mousePressEvent(QMouseEvent* event)
{
  last_mouse_x_ = event->x();
  last_mouse_y_ = event->y();
}

void RendererWindow::mouseMoveEvent(QMouseEvent* event)
{
  const int x = event->x();
  const int y = event->y();
  const int dx = x - last_mouse_x_;
  const int dy = y - last_mouse_y_;

  last_mouse_x_ = x;
  last_mouse_y_ = y;

  switch (event->buttons())
  {
    case Qt::LeftButton:
      camera_->rotatePixel(dx, dy);
      update();
      break;

    case Qt::RightButton:
      camera_->translatePixel(dx, dy);
      update();
      break;

    case Qt::LeftButton | Qt::RightButton:
      camera_->zoomPixel(dx, dy);
      update();
      break;
  }
}
}