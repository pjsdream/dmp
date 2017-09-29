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
#include <dmp/trajectory/cubic_spline_trajectory.h>
#include <dmp/trajectory/cubic_spline.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_custom_texture.h>
#include <dmp/rendering/request/request_custom_mesh.h>
#include <dmp/utils/mesh_loader.h>
#include <dmp/utils/rate.h>
#include <dmp/utils/timer.h>
#include <dmp/planning/objective/objective_grip.h>
#include <dmp/planning/objective/objective_reach_to_grip.h>

#include <iostream>

namespace dmp
{
Planner::Planner(const PlanningOption& option)
    : Node("planner")
{
  setRobotModel(option.getRobotModel());
  setEnvironment(option.getEnvironment());
  setMotion(option.getMotion());

  trajectory_duration_ = option.getTrajectoryDuration();
  trajectory_num_curves_ = option.numTrajectoryCurves();
  timestep_ = option.getTimestep();

  // initialize trajectory
  // TODO: change the joint names. For now, use the body joints only
  trajectory_ =
      std::make_unique<CubicSplineTrajectory>(motion_->getBodyJoints(), trajectory_duration_, trajectory_num_curves_);

  // drawing environment
  drawGround();
  drawEnvironment();
}

Planner::~Planner() = default;

Subscriber<RobotState>& Planner::getRobotStateSubscriber()
{
  return robot_state_subscriber_;
}

Subscriber<Objective>& Planner::getObjectiveSubscriber()
{
  return objective_subscriber_;
}

Publisher<Request>& Planner::getRendererPublisher()
{
  return renderer_publisher_;
}

Publisher<Trajectory>& Planner::getTrajectoryPublisher()
{
  return trajectory_publisher_;
}

void Planner::run()
{
  using namespace std::chrono_literals;

  const auto& body_joint_names = motion_->getBodyJoints();

  auto rate = Rate::withDuration(timestep_);

  while (!stopRequested())
  {
    // Assume that the previous planning step has been done in time.


    // Send the whole trajectory to the controller. The controller will overwrite the previously passed trajectory with
    // the new one. Takes less than 0.1 ms.
    constexpr auto trajectory_discretization = 30;
    Trajectory trajectory(body_joint_names);

    for (int i = 0; i < trajectory_discretization; i++)
    {
      const auto t = i * trajectory_duration_ / trajectory_discretization;

      Eigen::VectorXd joint_positions(body_joint_names.size());
      for (int j = 0; j < body_joint_names.size(); j++)
      {
        const auto& spline = trajectory_->getSpline(body_joint_names[j]);
        joint_positions(j) = spline.position(t);
      }

      TrajectoryPoint point(t, joint_positions);
      trajectory.addPoint(point);
    }

    trajectory_publisher_.publish(std::move(trajectory));


    // Receive the current robot state. Step forward the trajectory by timestep, finding the best fitting of spline
    // trajectory. Take about 2 ms.
    auto robot_state_requests = robot_state_subscriber_.popAll();
    std::unique_ptr<RobotState> robot_state_request;
    if (!robot_state_requests.empty())
      robot_state_request = std::move(*robot_state_requests.rbegin());

    stepForwardTrajectory(rate.duration(), std::move(robot_state_request));


    // Receive objectives
    auto objective_requests = objective_subscriber_.popAll();
    for (auto& objective : objective_requests)
    {
      objectives_.push_back(std::shared_ptr<Objective>(std::move(objective)));
      objective_completion_times_.push_back(trajectory_duration_);
    }


    // Optimize the trajectory within the time limit
    // For testing the controller, increasing the joint splines
    print("remaining planning time: %lf ms\n", rate.remainingTime() * 1000.);

    // Spend 0.9 x remaining time for optimization.
    optimize(rate.remainingTime() * 0.9);
    print("remaining time after planning: %lf ms\n", rate.remainingTime() * 1000.);

    // Test code for spline control.
    /*
    for (int i = 0; i < body_joint_names.size(); i++)
    {
      auto& spline = trajectory_->getSpline(body_joint_names[i]);
      for (int j = 0; j <= trajectory_num_curves_; j++)
      {
        spline.controlPosition(j) += 0.01 * j;
      }
    }
     */


    // TODO
    // Draw the current motion plan in the renderer.


    // Wait for the next timestep
    rate.sleep();
  }
}

void Planner::optimize(double remaining_time)
{
  Timer timer(remaining_time);

  while (!timer.isOver())
  {
    // Alternating optimization

    // TODO: optimize the trajectory

    // TODO: optimize over the objective completion time

    printf("Objective completion times:");
    for (int i=0; i<objective_completion_times_.size(); i++)
      printf(" %lf", objective_completion_times_[i]);
    printf("\n");
  }
}

void Planner::stepForwardTrajectory(double time, std::unique_ptr<RobotState> robot_state)
{
  // If it is given, then change the initial position and velocity of the new trajectory to the current robot state.
  // Otherwise, the initial position and velocity of the new trajectory should match with ones of the original
  // trajectory.

  for (const auto& joint_name : trajectory_->getJointNames())
  {
    auto& spline = trajectory_->getSpline(joint_name);

    double p0;
    double v0;

    if (robot_state)
    {
      p0 = robot_state->position(joint_name);
      v0 = robot_state->velocity(joint_name);
    }
    else
    {
      p0 = spline.position(time);
      v0 = spline.velocity(time);
    }

    // TODO: sampling
    constexpr auto num_samples_per_curve = 10;
    const auto total_samples = trajectory_num_curves_ * num_samples_per_curve;
    std::vector<std::tuple<double, double>> samples;

    for (int i = 0; i < total_samples; i++)
    {
      const auto t = trajectory_duration_ * i / (total_samples - 1);
      double p;

      if (t + time <= trajectory_duration_)
      {
        p = spline.position(t + time);
      }
      else
      {
        // Linear extrapolate
        const auto extra_time = t + time - trajectory_duration_;
        const auto v = spline.velocity(trajectory_duration_);
        p = spline.position(trajectory_duration_) + extra_time * v;
      }

      samples.push_back(std::make_tuple(t, p));
    }

    // Spline fitting
    spline.fitting(samples, p0, v0);
  }
}

void Planner::setRobotModel(const std::shared_ptr<RobotModel>& robot_model)
{
  robot_model_ = robot_model;

  // constructing bounding volumes
  int cnt = 0;
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
      cnt++;
    }
  }

  printf("%d bounding volumes\n", cnt);
}

void Planner::setMotion(const std::shared_ptr<Motion>& motion)
{
  motion_ = motion;
}

void Planner::setEnvironment(const std::shared_ptr<Environment>& environment)
{
  environment_ = environment;
}

void Planner::drawGround()
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

void Planner::drawRobotModel()
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

void Planner::drawEnvironment()
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
}
