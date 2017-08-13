#include <dmp/planning/planner.h>
#include <dmp/rendering/renderer.h>
#include <dmp/robot/robot_model.h>
#include <dmp/robot/robot_link.h>
#include <dmp/robot/robot_joint.h>

#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>

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
  void renderingTraverse(const std::shared_ptr<RobotLink>& link);

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

  auto link = robot_model_->getRoot();

  auto frame = std::make_unique<RequestFrame>();
  frame->action = RequestFrame::Action::Set;
  frame->name = link->getName();
  frame->transform = Eigen::Affine3d::Identity();
  renderer_->sendRequest(std::move(frame));

  renderingTraverse(link);
}

void Planner::Impl::renderingTraverse(const std::shared_ptr<RobotLink>& link)
{
  for (auto visual : link->getVisuals())
  {
    auto frame = std::make_unique<RequestFrame>();
    frame->action = RequestFrame::Action::Set;
    frame->name = link->getName() + "_" + visual.filename;
    frame->parent = link->getName();
    frame->transform = visual.transform;
    renderer_->sendRequest(std::move(frame));

    auto req = std::make_unique<RequestMesh>();
    req->action = RequestMesh::Action::Attach;
    req->filename = visual.filename;
    req->frame = link->getName();
    renderer_->sendRequest(std::move(req));
  }

  for (auto joint : link->getChildJoints())
  {
    auto frame = std::make_unique<RequestFrame>();
    frame->action = RequestFrame::Action::Set;
    frame->name = joint->getChildLink()->getName();
    frame->parent = link->getName();
    frame->transform = joint->getTransform(0.);

    renderer_->sendRequest(std::move(frame));

    renderingTraverse(joint->getChildLink());
  }
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
