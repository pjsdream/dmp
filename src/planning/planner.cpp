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
  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);

private:
  std::shared_ptr<Renderer> renderer_;
  std::shared_ptr<RobotModel> robot_model_;
};

void Planner::Impl::setRenderer(const std::shared_ptr<Renderer>& renderer)
{
  renderer_ = renderer;
}

void Planner::Impl::setRobotModel(const std::shared_ptr<RobotModel>& robot_model)
{
  robot_model_ = robot_model;
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

void Planner::setRobotModel(const std::shared_ptr<RobotModel>& robot_model)
{
  impl_->setRobotModel(robot_model);
}
}
