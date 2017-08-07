#include <dmp/rendering/renderer.h>
#include <dmp/rendering/renderer_impl.h>
#include <dmp/rendering/scene/scene_manager.h>

namespace dmp
{
Renderer::Renderer(QWidget* parent)
    : impl_(std::make_unique<RendererImpl>(parent)), scene_manager_(std::make_unique<SceneManager>())
{
  impl_->show();
  impl_->resize(800, 600);
  impl_->move(100, 100);
}

Renderer::~Renderer() = default;
}
