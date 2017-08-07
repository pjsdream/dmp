#include <dmp/gui/renderer.h>
#include <dmp/gui/renderer_impl.h>

namespace dmp
{
Renderer::Renderer(QWidget* parent)
    : impl_(std::make_unique<RendererImpl>(parent))
{
  impl_->show();
  impl_->resize(800, 600);
  impl_->move(100, 100);
}

Renderer::~Renderer() = default;
}
