#include <dmp/rendering/renderer.h>
#include <dmp/rendering/request/request_manager.h>
#include <dmp/rendering/request/request.h>
#include <dmp/rendering/resource/resource_manager.h>
#include <dmp/rendering/resource/resource_mesh.h>
#include <dmp/rendering/scene/scene_manager.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>
#include <dmp/rendering/shader/light_shader.h>

#include <QTimer>

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

  void sendRequest(Request&& request);

private:
  void handleRequest(const Request& request);
  void traverseScene(const std::shared_ptr<SceneNode>& node, const Eigen::Affine3d& transform);

  void setFrame(const std::string& name, const Eigen::Affine3d& transform);
  void setFrame(const std::string& name, const std::string& parent, const Eigen::Affine3d& transform);
  void attachMesh(const std::string& name, const std::string& filename);

  std::shared_ptr<GlFunctions> gl_;

  std::unique_ptr<SceneManager> scene_manager_;
  std::unique_ptr<ResourceManager> resource_manager_;
  std::unique_ptr<RequestManager> request_manager_;

  std::unique_ptr<LightShader> light_shader_;
};

Renderer::Renderer(QWidget* parent)
    : QOpenGLWidget(parent),
      impl_(std::make_unique<Impl>())
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

void Renderer::sendRequest(Request&& request)
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

Renderer::Impl::Impl()
{
}

void Renderer::Impl::sendRequest(Request&& request)
{
  request_manager_->addRequest(std::move(request));
}

void Renderer::Impl::paintGL()
{
  gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // update scene upon requests
  std::vector<Request> requests;
  request_manager_->pullRequests(requests);

  for (const auto& request : requests)
    handleRequest(request);

  // traverse scene
  light_shader_->start();
  light_shader_->loadProjection(Eigen::Affine3d::Identity());
  light_shader_->loadView(Eigen::Affine3d::Identity());
  traverseScene(scene_manager_->getRoot(), Eigen::Affine3d::Identity());
  light_shader_->end();
}

void Renderer::Impl::resizeGL(int w, int h)
{
  gl_->glViewport(0, 0, w, h);
}

void Renderer::Impl::initializeGL(QOpenGLContext* context)
{
  auto deleter = [](GlFunctions*){};
  gl_.reset(context->versionFunctions<GlFunctions>(), deleter);

  gl_->glClearColor(0.8f, 0.8f, 0.8f, 0.f);
  gl_->glEnable(GL_DEPTH_TEST);

  scene_manager_ = std::make_unique<SceneManager>();
  resource_manager_ = std::make_unique<ResourceManager>(gl_);
  request_manager_ = std::make_unique<RequestManager>();

  // shaders
  light_shader_ = std::make_unique<LightShader>(gl_);
}

void Renderer::Impl::handleRequest(const Request& request)
{
  auto json = request.getJson();
  printf("action: %s\n", json["action"].toString().c_str());

  if (json["action"].toString() == "set frame")
  {
    auto name = json["name"].toString();
    Eigen::Affine3d transform;
    for (int i=0; i<16; i++)
      transform.matrix()(i%4, i/4) = json["transform"][i].toDouble();
    if (json.containsKey("parent"))
    {
      auto parent = json["parent"].toString();
      setFrame(name, parent, transform);
    }
    else
    {
      setFrame(name, transform);
    }
  }
  else if (json["action"].toString() == "attach mesh")
  {
    auto name = json["name"].toString();
    auto filename = json["filename"].toString();
    Eigen::Matrix4d transform;
    attachMesh(name, filename);
  }
}

void Renderer::Impl::traverseScene(const std::shared_ptr<SceneNode>& node, const Eigen::Affine3d& transform)
{
  light_shader_->loadModel(transform);

  for (const auto& resource : node->getAttachedResources())
  {
    // TODO: divide into multiple subclasses
    std::dynamic_pointer_cast<ResourceMesh>(resource)->draw();
  }

  for (const auto& edge : node->getEdges())
  {
    traverseScene(edge.getChild(), transform * edge.getTransform());
  }
}

void Renderer::Impl::setFrame(const std::string& name, const Eigen::Affine3d& transform)
{
  auto node = scene_manager_->createNode(name);
  auto root = scene_manager_->getRoot();
  root->createEdge(node, transform);
}
void Renderer::Impl::setFrame(const std::string& name, const std::string& parent, const Eigen::Affine3d& transform)
{
  auto node = scene_manager_->createNode(name);
  auto parent_node = scene_manager_->createNode(parent);
  parent_node->createEdge(node, transform);
}
void Renderer::Impl::attachMesh(const std::string& name, const std::string& filename)
{
  auto node = scene_manager_->getNode(name);
  if (node != nullptr)
    node->attachResource(resource_manager_->getMesh(filename));
}
}