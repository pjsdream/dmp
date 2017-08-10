#include <dmp/planning/planner.h>
#include <dmp/rendering/renderer.h>

namespace dmp
{
class Planner::Impl
{
public:
  Impl() = default;
  ~Impl() = default;

  void setRenderer(const std::shared_ptr<Renderer>& renderer);

private:
  std::shared_ptr<Renderer> renderer_;
};

void Planner::Impl::setRenderer(const std::shared_ptr<Renderer>& renderer)
{
  renderer_ = renderer;
}

Planner::Planner()
    : impl_(std::make_unique<Impl>())
{
}

Planner::~Planner() = default;

void Planner::setRenderer(const std::shared_ptr<Renderer>& renderer)
{
  impl_->setRenderer(renderer);
}
}
