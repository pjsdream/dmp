#include <dmp/planning/planner.h>
#include <dmp/rendering/renderer.h>
#include <dmp/robot/robot_model.h>
#include <dmp/robot/robot_link.h>
#include <dmp/robot/robot_joint.h>

#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_custom_texture.h>
#include <dmp/rendering/request/request_custom_mesh.h>

namespace dmp
{
class Planner::Impl
{
public:
  Impl() = default;
  ~Impl() = default;

  void setRenderer(const std::shared_ptr<Renderer>& renderer);
  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);
  void setEnvironment(const std::shared_ptr<Environment>& environment);

private:
  void renderingTraverse(const std::shared_ptr<RobotLink>& link);

  std::shared_ptr<Renderer> renderer_;
  std::shared_ptr<RobotModel> robot_model_;
};

void Planner::Impl::setRenderer(const std::shared_ptr<Renderer>& renderer)
{
  renderer_ = renderer;

  // draw ground
  auto frame = std::make_unique<RequestFrame>();
  frame->name = "ground";
  frame->transform = Eigen::Affine3d::Identity();
  renderer_->sendRequest(std::move(frame));

  auto custom_texture = std::make_unique<RequestCustomTexture>();
  custom_texture->name = "checkerboard";
  custom_texture->w = 2;
  custom_texture->h = 2;
  custom_texture->image = {192, 192, 192, 255, 255, 255, 255, 255, 255, 255, 255, 255, 192, 192, 192, 255};
  renderer_->sendRequest(std::move(custom_texture));

  auto custom_mesh = std::make_unique<RequestCustomMesh>();
  custom_mesh->name = "ground";
  custom_mesh->frame = "ground";
  custom_mesh->vertex_buffer = {-10, -10, 0, 10, -10, 0, 10, 10, 0, -10, 10, 0};
  custom_mesh->normal_buffer = {0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1};
  custom_mesh->texture_buffer = {-5, -5, 5, -5, 5, 5, -5, 5};
  custom_mesh->face_buffer = {0, 1, 2, 0, 2, 3};
  custom_mesh->texture_name = "checkerboard";
  renderer_->sendRequest(std::move(custom_mesh));
}

void Planner::Impl::setRobotModel(const std::shared_ptr<RobotModel>& robot_model)
{
  robot_model_ = robot_model;

  // draw robot model
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
