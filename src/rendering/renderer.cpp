#include <dmp/rendering/renderer.h>
#include <dmp/rendering/request/request_manager.h>
#include <dmp/rendering/request/request.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
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

  void sendRequest(std::unique_ptr<Request> request);

private:
  void registerRequestHandlers();

  void handleRequest(std::unique_ptr<Request> request);
  void handleRequestFrame(std::unique_ptr<RequestFrame> request);
  void handleRequestMesh(std::unique_ptr<RequestMesh> request);

  void traverseScene(const std::shared_ptr<SceneNode>& node, const Eigen::Affine3d& transform);

  std::shared_ptr<GlFunctions> gl_;

  std::unique_ptr<SceneManager> scene_manager_;
  std::unique_ptr<ResourceManager> resource_manager_;
  std::unique_ptr<RequestManager> request_manager_;

  std::unique_ptr<LightShader> light_shader_;

  std::unordered_map<const std::type_info*, std::function<void(std::unique_ptr<Request>)>> request_handlers_;
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

Renderer::Impl::Impl()
{
  registerRequestHandlers();
}

void Renderer::Impl::registerRequestHandlers()
{
  request_handlers_[&typeid(RequestFrame)] = [this](auto request){ handleRequestFrame(std::unique_ptr<RequestFrame>(dynamic_cast<RequestFrame*>(request.release()))); };
  request_handlers_[&typeid(RequestMesh)] = [this](auto request){ handleRequestMesh(std::unique_ptr<RequestMesh>(dynamic_cast<RequestMesh*>(request.release()))); };
}

void Renderer::Impl::handleRequest(std::unique_ptr<Request> request)
{
  request_handlers_[&typeid(*request)](std::move(request));
}

void Renderer::Impl::handleRequestFrame(std::unique_ptr<RequestFrame> request)
{
  scene_manager_->setFrame(request->parent, request->name, request->transform)
}

void Renderer::Impl::handleRequestMesh(std::unique_ptr<RequestMesh> request)
{
  scene_manager_->attachResource(request->frame, resource_manager_->getMesh(request->filename));
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
}