#include <dmp/planning/planner.h>
#include <dmp/planning/planning_option.h>
#include <dmp/rendering/renderer.h>
#include <dmp/robot/robot_model.h>
#include <dmp/robot/robot_link.h>
#include <dmp/robot/robot_joint.h>
#include <dmp/planning/environment/environment.h>
#include <dmp/planning/environment/object.h>
#include <dmp/planning/motion/motion.h>
#include <dmp/shape/shape.h>
#include <dmp/shape/cube.h>
#include <dmp/shape/cylinder.h>
#include <dmp/shape/sphere.h>

#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_custom_texture.h>
#include <dmp/rendering/request/request_custom_mesh.h>

namespace dmp
{
class Planner::Impl
{
public:
  Impl() = delete;
  explicit Impl(const PlanningOption& option);
  ~Impl() = default;

  Impl(const Impl& rhs) = delete;
  Impl& operator = (const Impl& rhs) = delete;

  Impl(Impl&& rhs) = delete;
  Impl& operator = (Impl&& rhs) = delete;

  void plan();

private:
  void renderingTraverse(const std::shared_ptr<RobotLink>& link);

  void setRenderer(const std::shared_ptr<Renderer>& renderer);
  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);
  void setMotion(const std::shared_ptr<Motion>& motion);
  void setEnvironment(const std::shared_ptr<Environment>& environment);

  std::shared_ptr<Renderer> renderer_;
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<Environment> environment_;
  std::shared_ptr<Motion> motion_;
};

Planner::Impl::Impl(const PlanningOption& option)
{
  setRenderer(option.getRenderer());
  setRobotModel(option.getRobotModel());
  setEnvironment(option.getEnvironment());
  setMotion(option.getMotion());
}

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
  const auto& link = robot_model_->getRoot();

  auto frame = std::make_unique<RequestFrame>();
  frame->action = RequestFrame::Action::Set;
  frame->name = link->getName();
  frame->transform = Eigen::Affine3d::Identity();
  renderer_->sendRequest(std::move(frame));

  renderingTraverse(link);
}

void Planner::Impl::renderingTraverse(const std::shared_ptr<RobotLink>& link)
{
  for (const auto& visual : link->getVisuals())
  {
    auto frame = std::make_unique<RequestFrame>();
    frame->action = RequestFrame::Action::Set;
    frame->name = link->getName() + ":" + visual.filename;
    frame->parent = link->getName();
    frame->transform = visual.transform;
    renderer_->sendRequest(std::move(frame));

    auto req = std::make_unique<RequestMesh>();
    req->action = RequestMesh::Action::Attach;
    req->filename = visual.filename;
    req->frame = link->getName() + ":" + visual.filename;
    renderer_->sendRequest(std::move(req));
  }

  for (const auto& joint : link->getChildJoints())
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

void Planner::Impl::setMotion(const std::shared_ptr<Motion>& motion)
{
  motion_ = motion;
}

void Planner::Impl::setEnvironment(const std::shared_ptr<Environment>& environment)
{
  environment_ = environment;

  // drawing environment
  const auto& objects = environment_->getObjects();
  for (const auto& object : objects)
  {
    const auto& shape = object->getShape();
    const auto& color = object->getColor();

    // frame
    auto frame = std::make_unique<RequestFrame>();
    frame->action = RequestFrame::Action::Set;
    frame->name = "object_" + std::to_string(std::rand());
    frame->transform = Eigen::Affine3d::Identity();

    // shape
    auto custom_mesh = std::make_unique<RequestCustomMesh>();
    custom_mesh->name = frame->name;
    custom_mesh->frame = frame->name;

    if (auto cube = dynamic_cast<Cube*>(shape.get()))
    {
      custom_mesh->createCube(cube->getSize());
      custom_mesh->setGlobalColor(Eigen::Vector3f(color(0), color(1), color(2)));

      frame->transform = cube->getTransform();
    }
    else if (auto cylinder = dynamic_cast<Cylinder*>(shape.get()))
    {
      custom_mesh->createCylinder(cylinder->getRadius(), cylinder->getHeight());
      custom_mesh->setGlobalColor(Eigen::Vector3f(color(0), color(1), color(2)));

      frame->transform = cylinder->getTransform();
    }
    else if (auto sphere = dynamic_cast<Sphere*>(shape.get()))
    {
      custom_mesh->createSphere(sphere->getRadius());
      custom_mesh->setGlobalColor(Eigen::Vector3f(color(0), color(1), color(2)));

      frame->transform.translate(sphere->getPosition());
    }

    renderer_->sendRequest(std::move(frame));
    renderer_->sendRequest(std::move(custom_mesh));
  }
}

void Planner::Impl::plan()
{
}

Planner::Planner(const PlanningOption& option)
    : impl_(std::make_unique<Impl>(option))
{
}

Planner::~Planner() = default;

void Planner::plan()
{
  impl_->plan();
}
}
