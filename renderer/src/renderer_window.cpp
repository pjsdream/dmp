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
#include <renderer/resource/mesh_loader.h>

#include <QMouseEvent>

#include <iostream>

namespace dmp
{
//
// RendererWindow::Impl
//
RendererWindow::RendererWindow()
    : camera_(std::make_unique<Camera>()),
      scene_manager_(std::make_unique<SceneManager>()),
      light_manager_(std::make_unique<LightManager>())
{
}

RendererWindow::~RendererWindow() = default;

/*
void RendererWindow::handleRequest(std::shared_ptr<Request> request)
{
  handleRequestFrame(std::dynamic_pointer_cast<RequestFrame>(request));
  handleRequestMesh(std::dynamic_pointer_cast<RequestMesh>(request));
  handleRequestLight(std::dynamic_pointer_cast<RequestLight>(request));
  handleRequestCustomTexture(std::dynamic_pointer_cast<RequestCustomTexture>(request));
  handleRequestCustomMesh(std::dynamic_pointer_cast<RequestCustomMesh>(request));
}

void RendererWindow::handleRequestFrame(std::shared_ptr<RequestFrame> request)
{
  if (request == nullptr)
    return;

  scene_manager_->setFrame(request->parent, request->name, request->transform);
}

void RendererWindow::handleRequestMesh(std::shared_ptr<RequestMesh> request)
{
  if (request == nullptr)
    return;

  scene_manager_->attachResource(request->frame, resource_manager_->getMesh(request->filename));
}

void RendererWindow::handleRequestLight(std::shared_ptr<RequestLight> request)
{
  if (request == nullptr)
    return;

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

void RendererWindow::handleRequestCustomTexture(std::shared_ptr<RequestCustomTexture> request)
{
  if (request == nullptr)
    return;

  TextureLoaderRawTexture texture;
  texture.width = request->w;
  texture.height = request->h;
  texture.image = std::move(request->image);

  resource_manager_->createTexture(request->name, std::move(texture));
}

void RendererWindow::handleRequestCustomMesh(std::shared_ptr<RequestCustomMesh> request)
{
  if (request == nullptr)
    return;

  MeshLoaderRawMesh raw_mesh;
  raw_mesh.vertex_buffer = std::move(request->vertex_buffer);
  raw_mesh.normal_buffer = std::move(request->normal_buffer);
  raw_mesh.texture_buffer = std::move(request->texture_buffer);
  raw_mesh.color_buffer = std::move(request->color_buffer);
  raw_mesh.face_buffer = std::move(request->face_buffer);
  raw_mesh.has_global_color = request->hasGlobalColor();
  raw_mesh.global_color = request->getGlobalColor();

  auto mesh = resource_manager_->createMesh(request->name, std::move(raw_mesh));

  if (!request->texture_name.empty())
  {
    auto texture = resource_manager_->getTexture(request->texture_name);
    mesh->setTexture(texture);
  }

  scene_manager_->attachResource(request->frame, mesh);
}
 */

void RendererWindow::paintGL()
{
  gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // TODO: Update scene upon requests

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