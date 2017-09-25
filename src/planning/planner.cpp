#include <dmp/planning/planner.h>
#include <dmp/planning/planning_option.h>
#include <dmp/robot/robot_model.h>
#include <dmp/robot/robot_link.h>
#include <dmp/robot/robot_joint.h>
#include <dmp/planning/environment/environment.h>
#include <dmp/planning/environment/object.h>
#include <dmp/planning/motion/motion.h>
#include <dmp/shape/aabb.h>
#include <dmp/shape/cube.h>
#include <dmp/shape/cylinder.h>
#include <dmp/shape/sphere.h>
#include <dmp/shape/bounding_volume_factory.h>

#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_custom_texture.h>
#include <dmp/rendering/request/request_custom_mesh.h>

#include <dmp/utils/mesh_loader.h>
#include <dmp/utils/rate.h>

#include <iostream>

namespace dmp
{
class Planner::Impl
{
public:
  Impl() = delete;
  explicit Impl(const PlanningOption& option);
  ~Impl() = default;

  Impl(const Impl& rhs) = delete;
  Impl& operator=(const Impl& rhs) = delete;

  Impl(Impl&& rhs) = delete;
  Impl& operator=(Impl&& rhs) = delete;

  void run();

  Publisher<Request>& getRendererPublisher();
  Publisher<Trajectory>& getTrajectoryPublisher();

private:
  void drawGround();
  void drawRobotModel();
  void drawEnvironment();

  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);
  void setMotion(const std::shared_ptr<Motion>& motion);
  void setEnvironment(const std::shared_ptr<Environment>& environment);

  Publisher<Request> renderer_publisher_;
  Publisher<Trajectory> trajectory_publisher_;

  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<Environment> environment_;
  std::shared_ptr<Motion> motion_;

  std::vector<std::vector<std::shared_ptr<Shape>>> bounding_volumes_;
};

Planner::Impl::Impl(const PlanningOption& option)
{
  setRobotModel(option.getRobotModel());
  setEnvironment(option.getEnvironment());
  setMotion(option.getMotion());

  // drawing environment
  drawGround();
  drawEnvironment();
}

Publisher<Request>& Planner::Impl::getRendererPublisher()
{
  return renderer_publisher_;
}

Publisher<Trajectory>& Planner::Impl::getTrajectoryPublisher()
{
  return trajectory_publisher_;
}

void Planner::Impl::run()
{
  using namespace std::chrono_literals;

  const auto& joint_names = motion_->getBodyJoints();

  // 20Hz re-planning
  Rate rate(20);

  for (int i = 0; i < 20 * 100; i++)
  {
    // generate a random trajectory
    Trajectory trajectory(joint_names);

    for (int j = 0; j < 30; j++)
    {
      const double t = j * 0.1;

      Eigen::VectorXd joint_positions(joint_names.size());
      for (int k = 0; k < joint_names.size(); k++)
      {
        const auto& joint = robot_model_->getJoint(joint_names[k]);
        joint_positions(k) = (std::rand() / 2147483647.) * (joint.getUpper() - joint.getLower()) + joint.getLower();
      }

      TrajectoryPoint point(t, joint_positions);
      trajectory.addPoint(point);
    }

    trajectory_publisher_.publish(std::move(trajectory));

    // running at 20Hz
    rate.sleep();
  }
}

void Planner::Impl::setRobotModel(const std::shared_ptr<RobotModel>& robot_model)
{
  robot_model_ = robot_model;

  // constructing bounding volumes
  const auto num_links = robot_model_->numLinks();
  for (int i = 0; i < num_links; i++)
  {
    const auto& link = robot_model_->getLink(i);

    // TODO: refactoring the code. better storage for bounding volumes
    bounding_volumes_.emplace_back(std::vector<std::shared_ptr<Shape>>());

    for (const auto& collision : link.getCollisions())
    {
      auto future_raw_mesh = MeshLoader::loadMesh(collision.filename);
      const auto& raw_mesh = future_raw_mesh.get();

      const auto num_vertices = raw_mesh.vertex_buffer.size() / 3;

      // allocate vertex vector
      VectorEigen<Eigen::Vector3d> points(num_vertices);
      for (int j = 0; j < num_vertices; j++)
      {
        points[j] = collision.transform * Eigen::Vector3d(raw_mesh.vertex_buffer[3 * j],
                                                          raw_mesh.vertex_buffer[3 * j + 1],
                                                          raw_mesh.vertex_buffer[3 * j + 2]);
      }

      // create bounding volume through factory function
      auto bounding_volume = BoundingVolumeFactory::newBoundingVolumeFromPoints(points);

      // TODO: save computed bounding volume to a storage
      bounding_volumes_[i].push_back(bounding_volume);
    }
  }
}

void Planner::Impl::setMotion(const std::shared_ptr<Motion>& motion)
{
  motion_ = motion;
}

void Planner::Impl::setEnvironment(const std::shared_ptr<Environment>& environment)
{
  environment_ = environment;
}

void Planner::Impl::drawGround()
{
  // draw ground
  auto frame = std::make_unique<RequestFrame>();
  frame->name = "ground";
  frame->transform = Eigen::Affine3d::Identity();
  renderer_publisher_.publish(std::move(frame));

  auto custom_texture = std::make_unique<RequestCustomTexture>();
  custom_texture->name = "checkerboard";
  custom_texture->w = 2;
  custom_texture->h = 2;
  custom_texture->image = {192, 192, 192, 255, 255, 255, 255, 255, 255, 255, 255, 255, 192, 192, 192, 255};
  renderer_publisher_.publish(std::move(custom_texture));

  auto custom_mesh = std::make_unique<RequestCustomMesh>();
  custom_mesh->name = "ground";
  custom_mesh->frame = "ground";
  custom_mesh->vertex_buffer = {-10, -10, 0, 10, -10, 0, 10, 10, 0, -10, 10, 0};
  custom_mesh->normal_buffer = {0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1};
  custom_mesh->texture_buffer = {-5, -5, 5, -5, 5, 5, -5, 5};
  custom_mesh->face_buffer = {0, 1, 2, 0, 2, 3};
  custom_mesh->texture_name = "checkerboard";
  renderer_publisher_.publish(std::move(custom_mesh));
}

void Planner::Impl::drawRobotModel()
{
  auto num_links = robot_model_->numLinks();

  for (int i = 0; i < num_links; i++)
  {
    const auto& link = robot_model_->getLink(i);

    // frame
    // TODO: send requests at once. Currently, it's sending one by one.
    const std::string frame_name{std::string("model_") + link.getName()};
    auto frame = std::make_unique<RequestFrame>();
    frame->action = RequestFrame::Action::Set;
    frame->name = frame_name;
    if (i == 0)
      frame->transform = Eigen::Affine3d::Identity();
    else
    {
      const auto& parent = robot_model_->getParent(i);
      const auto& parent_link = robot_model_->getLink(parent);
      const auto& joint = robot_model_->getJoint(i);

      frame->parent = std::string("model_") + parent_link.getName();
      frame->transform = joint.getJointTransform((joint.getLower() + joint.getUpper()) / 2.);
    }
    renderer_publisher_.publish(std::move(frame));

    // mesh requests
    // TODO: send requests at once. Currently, it's sending one by one.
    for (const auto& visual : link.getVisuals())
    {
      auto frame = std::make_unique<RequestFrame>();
      frame->action = RequestFrame::Action::Set;
      frame->name = frame_name + "_" + visual.filename;
      frame->parent = frame_name;
      frame->transform = visual.transform;
      renderer_publisher_.publish(std::move(frame));

      auto mesh = std::make_unique<RequestMesh>();
      mesh->action = RequestMesh::Action::Attach;
      mesh->filename = visual.filename;
      mesh->frame = frame_name + "_" + visual.filename;
      renderer_publisher_.publish(std::move(mesh));
    }

    // collision mesh requests
    // TODO: send requests at once. Currently, it's sending one by one.
    for (const auto& collision : link.getCollisions())
    {
      auto frame = std::make_unique<RequestFrame>();
      frame->action = RequestFrame::Action::Set;
      frame->name = frame_name + "_" + collision.filename;
      frame->parent = frame_name;
      frame->transform = collision.transform;
      renderer_publisher_.publish(std::move(frame));

      auto mesh = std::make_unique<RequestMesh>();
      mesh->action = RequestMesh::Action::Attach;
      mesh->filename = collision.filename;
      mesh->frame = frame_name + "_" + collision.filename;
      renderer_publisher_.publish(std::move(mesh));
    }

    // TODO: refactoring the code
    // TODO: collision shape requests
    for (int j = 0; j < bounding_volumes_[i].size(); j++)
    {
      const auto& aabb = bounding_volumes_[i][j]->as<AABB>();

      auto frame = std::make_unique<RequestFrame>();
      frame->action = RequestFrame::Action::Set;
      frame->name = frame_name + "_" + "aabb_" + std::to_string(i) + "_" + std::to_string(j);
      frame->parent = frame_name;
      frame->transform = Eigen::Affine3d::Identity();
      frame->transform.translate((aabb.getMin() + aabb.getMax()) * .5);
      renderer_publisher_.publish(std::move(frame));

      auto custom_mesh = std::make_unique<RequestCustomMesh>();
      custom_mesh->name = "aabb_" + std::to_string(i) + "_" + std::to_string(j);
      custom_mesh->frame = frame_name + "_" + "aabb_" + std::to_string(i) + "_" + std::to_string(j);
      custom_mesh->createCube(aabb.getMax() - aabb.getMin());
      custom_mesh->setGlobalColor(Eigen::Vector3f(0.8f, 0.8f, 0.8f));
      renderer_publisher_.publish(std::move(custom_mesh));
    }
  }
}

void Planner::Impl::drawEnvironment()
{
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

    renderer_publisher_.publish(std::move(frame));
    renderer_publisher_.publish(std::move(custom_mesh));
  }
}

//
// Planner to pImpl
//
Planner::Planner(const PlanningOption& option)
    : Node("planner"), impl_(std::make_unique<Impl>(option))
{
}

Planner::~Planner() = default;

Publisher<Request>& Planner::getRendererPublisher()
{
  return impl_->getRendererPublisher();
}

Publisher<Trajectory>& Planner::getTrajectoryPublisher()
{
  return impl_->getTrajectoryPublisher();
}

void Planner::run()
{
  impl_->run();
}
}
