#include <dmp/rendering/renderer.h>
#include <dmp/rendering/request/request_manager.h>
#include <dmp/rendering/resource/resource_manager.h>
#include <dmp/rendering/resource/resource_mesh.h>
#include <dmp/rendering/scene/scene_manager.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>
#include <dmp/rendering/scene/scene_object.h>
#include <dmp/rendering/scene/scene_mesh_object.h>
#include <dmp/rendering/shader/light_shader.h>

#include <QTimer>

namespace dmp
{
Renderer::Renderer(QWidget* parent)
    : QOpenGLWidget(parent),
      gl_(std::make_shared<GlFunctions>())
{
  resize(800, 600);
  move(100, 100);
  show();

  QTimer* timer = new QTimer(this);
  timer->setInterval(16);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  timer->start();
}

Renderer::~Renderer() = default;

void Renderer::paintGL()
{
  gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // traverse scene
  light_shader_->start();
  light_shader_->loadProjection(Eigen::Affine3d::Identity());
  light_shader_->loadView(Eigen::Affine3d::Identity());
  traverseScene(scene_manager_->getRoot(), Eigen::Affine3d::Identity());
  light_shader_->end();
}

void Renderer::traverseScene(const std::shared_ptr<SceneNode>& node, const Eigen::Affine3d& transform)
{
  light_shader_->loadModel(transform);

  for (const auto& object : node->getAttachedObjects())
  {
    if (object->isMeshObject())
    {
      const auto& filename = std::dynamic_pointer_cast<SceneMeshObject>(object)->getFilename();

      auto mesh = resource_manager_->getMesh(filename);
      mesh->draw();
    }
  }

  for (const auto& edge : node->getEdges())
  {
    traverseScene(edge.getChild(), transform * edge.getTransform());
  }
}

void Renderer::resizeGL(int w, int h)
{
  gl_->glViewport(0, 0, w, h);
}

void Renderer::initializeGL()
{
  auto deleter = [](QOpenGLFunctions_4_1_Core*){};
  gl_.reset(context()->versionFunctions<QOpenGLFunctions_4_1_Core>(), deleter);

  gl_->glClearColor(0.8f, 0.8f, 0.8f, 0.f);
  gl_->glEnable(GL_DEPTH_TEST);

  scene_manager_ = std::make_unique<SceneManager>();
  resource_manager_ = std::make_unique<ResourceManager>(gl_);
  request_manager_ = std::make_unique<RequestManager>();

  // shaders
  light_shader_ = std::make_unique<LightShader>(gl_);
}
}