#include <dmp/rendering/renderer.h>

namespace dmp
{
Renderer::Renderer(QWidget* parent)
    : QOpenGLWidget(parent), scene_manager_(std::make_unique<SceneManager>())
{
  resize(800, 600);
  move(100, 100);
  show();
}

void Renderer::paintGL()
{
  gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::resizeGL(int w, int h)
{
  gl_->glViewport(0, 0, w, h);
}

void Renderer::initializeGL()
{
  initializeBaseGL(context());

  gl_->glClearColor(0.8f, 0.8f, 0.8f, 0.f);
}

std::shared_ptr<SceneNode> Renderer::createNode()
{
  return scene_manager_->createNode();
}

std::shared_ptr<SceneNode> Renderer::createNode(const Eigen::Affine3d& transform)
{
  return scene_manager_->createNode(transform);
}

std::shared_ptr<SceneNode> Renderer::createNode(const Eigen::Vector3d& translate)
{
  return scene_manager_->createNode(translate);
}

std::shared_ptr<SceneObject> Renderer::createMeshObject(const std::string& filename)
{
  return scene_manager_->createMeshObject(filename);
}
}