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
#include <dmp/trajectory/cubic_spline_trajectory.h>
#include <dmp/trajectory/cubic_spline.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/rendering/request/request_custom_texture.h>
#include <dmp/rendering/request/request_custom_mesh.h>
#include <dmp/utils/mesh_loader.h>
#include <dmp/utils/rate.h>
#include <dmp/utils/timer.h>
#include <dmp/utils/stopwatch.h>
#include <dmp/planning/objective/objective_grip.h>
#include <dmp/planning/objective/objective_reach_to_grip.h>
#include <dmp/planning/cost/cost.h>
#include <dmp/robot/robot_configuration.h>
#include <dmp/robot/planning_robot_model.h>
#include <dmp/robot/planning_robot_joint.h>
#include <dmp/robot/planning_robot_link.h>

#include <iostream>

namespace dmp
{
Planner::Planner(const std::shared_ptr<Manager>& manager, const PlanningOption& option)
    : Node(manager, "planner")
{
  robot_state_subscriber_ = createSubscriber<RobotState>("robot_state");
  objective_subscriber_ = createSubscriber<Objective>("objective");
  cost_subscriber_ = createSubscriber<Cost>("cost");
  renderer_publisher_ = createPublisher<Request>("rendering");
  trajectory_publisher_ = createPublisher<Trajectory>("trajectory");

  setRobotModel(option.getRobotModel());
  environment_ = option.getEnvironment();
  motion_ = option.getMotion();

  createPlanningRobotModel();

  trajectory_duration_ = option.getTrajectoryDuration();
  trajectory_num_curves_ = option.numTrajectoryCurves();
  timestep_ = option.getTimestep();
  discretizations_ = option.getDiscretizations();

  // Initialize trajectory
  // TODO: change the joint names. For now, use the body joints only
  trajectory_ = std::make_unique<CubicSplineTrajectory>(motion_->getBodyJoints(), trajectory_num_curves_);

  // TODO: initial trajectory pose
  trajectory_->getSpline(0).controlPosition(0) = 0.3;
  trajectory_->getSpline(1).controlPosition(0) = 0.3;
  trajectory_->getSpline(2).controlPosition(0) = -0.6;
  trajectory_->getSpline(3).controlPosition(0) = 0.0;
  trajectory_->getSpline(4).controlPosition(0) = 1.1;
  trajectory_->getSpline(5).controlPosition(0) = 0.0;
  trajectory_->getSpline(6).controlPosition(0) = 1.07;
  trajectory_->getSpline(7).controlPosition(0) = 0.3;

  // Allocate robot configurations
  const int num_max_objectives = 10;
  configurations_ =
      std::vector<RobotConfiguration>(discretizations_ + num_max_objectives, RobotConfiguration(planning_robot_model_));

  // Drawing environment
  drawGround();
  drawEnvironment();

  Eigen::VectorXd p(8);
  p.setZero();
  //drawRobotStatus(p);
}

Planner::~Planner() = default;

void Planner::createPlanningRobotModel()
{
  planning_robot_model_ = std::make_shared<PlanningRobotModel>(robot_model_, motion_);
}

void Planner::run()
{
  using namespace std::chrono_literals;

  const auto& body_joint_names = motion_->getBodyJoints();

  // Open finger
  auto joint_names = body_joint_names;
  joint_names.push_back("r_gripper_finger_joint");
  joint_names.push_back("l_gripper_finger_joint");

  auto rate = Rate::withDuration(timestep_);

  while (!stopRequested())
  {
    // Assume that the previous planning step has been done in time.


    // Send the whole trajectory to the controller. The controller will overwrite the previously passed trajectory with
    // the new one. Takes less than 0.1 ms.
    constexpr auto trajectory_discretization = 30;
    Trajectory trajectory(joint_names);

    for (int i = 0; i < trajectory_discretization; i++)
    {
      const auto t = static_cast<double>(i) / trajectory_discretization * trajectory_num_curves_;

      Eigen::VectorXd joint_positions(body_joint_names.size() + 2);
      for (int j = 0; j < body_joint_names.size(); j++)
      {
        const auto& spline = trajectory_->getSpline(body_joint_names[j]);
        joint_positions(j) = spline.position(t);
      }

      joint_positions(body_joint_names.size()) = 0.05;
      joint_positions(body_joint_names.size() + 1) = 0.05;

      TrajectoryPoint point(t, joint_positions);
      trajectory.addPoint(point);
    }

    trajectory_publisher_.publish(std::move(trajectory));


    // Receive the current robot state. Step forward the trajectory by timestep, finding the best fitting of spline
    // trajectory. Take about 2 ms.
    // TODO: refactoring comm
    auto robot_state_requests = robot_state_subscriber_.popAll();
    std::shared_ptr<RobotState> robot_state_request;
    if (!robot_state_requests.empty())
      robot_state_request = *robot_state_requests.rbegin();

    //stepForwardTrajectory(rate.duration(), std::move(robot_state_request));


    // Receive objectives
    auto objective_requests = objective_subscriber_.popAll();
    for (auto& objective : objective_requests)
    {
      objectives_.push_back(std::shared_ptr<Objective>(std::move(objective)));
      objective_completion_times_.push_back(trajectory_duration_);
    }


    // Receive costs
    auto cost_requests = cost_subscriber_.popAll();
    for (auto& cost : cost_requests)
      costs_.push_back(std::shared_ptr<Cost>(std::move(cost)));


    // Optimize the trajectory within the time limit
    // For testing the controller, increasing the joint splines
    print("remaining planning time: %lf ms\n", rate.remainingTime() * 1000.);

    // Spend 0.9 x remaining time for optimization.
    optimize(rate.remainingTime() * 0.9);
    print("remaining time after planning: %lf ms\n", rate.remainingTime() * 1000.);

    // Test code for spline control.
    for (int j = 0; j <= trajectory_num_curves_; j++)
    {
      /*
      trajectory_->getSpline("torso_lift_joint").controlPosition(j) = 0.3;
      trajectory_->getSpline("shoulder_pan_joint").controlPosition(j) = -0.3;
      trajectory_->getSpline("shoulder_lift_joint").controlPosition(j) = -0.8;
      trajectory_->getSpline("upperarm_roll_joint").controlPosition(j) = 0.0;
      trajectory_->getSpline("elbow_flex_joint").controlPosition(j) = 1.0;
      trajectory_->getSpline("forearm_roll_joint").controlPosition(j) = 0.0;
      trajectory_->getSpline("wrist_flex_joint").controlPosition(j) = 1.37;
      trajectory_->getSpline("wrist_roll_joint").controlPosition(j) = -0.3;
      */
      /*
      trajectory_->getSpline("torso_lift_joint").controlPosition(j) = 0.3;
      trajectory_->getSpline("shoulder_pan_joint").controlPosition(j) = 0.3;
      trajectory_->getSpline("shoulder_lift_joint").controlPosition(j) = -0.6;
      trajectory_->getSpline("upperarm_roll_joint").controlPosition(j) = 0.0;
      trajectory_->getSpline("elbow_flex_joint").controlPosition(j) = 1.1;
      trajectory_->getSpline("forearm_roll_joint").controlPosition(j) = 0.0;
      trajectory_->getSpline("wrist_flex_joint").controlPosition(j) = 1.07;
      trajectory_->getSpline("wrist_roll_joint").controlPosition(j) = 0.3;
       */

      /*
      const double t = static_cast<double>(j) / trajectory_num_curves_;
      trajectory_->getSpline("torso_lift_joint").controlPosition(j) = 0.3 * (1 - t) + 0.3 * t;
      trajectory_->getSpline("shoulder_pan_joint").controlPosition(j) = 0.3 * (1 - t) + -0.3 * t;
      trajectory_->getSpline("shoulder_lift_joint").controlPosition(j) = -0.6 * (1 - t) + -0.8 * t;
      trajectory_->getSpline("upperarm_roll_joint").controlPosition(j) = 0.0;
      trajectory_->getSpline("elbow_flex_joint").controlPosition(j) = 1.1 * (1 - t) + 1.0 * t;
      trajectory_->getSpline("forearm_roll_joint").controlPosition(j) = 0.0;
      trajectory_->getSpline("wrist_flex_joint").controlPosition(j) = 1.07 * (1 - t) + 1.37 * t;
      trajectory_->getSpline("wrist_roll_joint").controlPosition(j) = 0.3 * (1 - t) + -0.3 * t;
       */
    }

    // Draw the current motion plan in the renderer.
    drawTrajectory();


    // Wait for the next timestep
    rate.sleep();
  }
}

void Planner::optimize(double remaining_time)
{
  Timer timer(remaining_time);

  const auto& joint_names = trajectory_->getJointNames();

  // Because the initial control doesn't change while optimizing the trajectory, the number of optimizing control points
  // is the same as the number of curves.
  const auto num_control_points = trajectory_->numCurves();

  while (!timer.isOver())
  {
    Stopwatch stopwatch;

    // Alternating optimization

    // TODO: optimize the trajectory
    // Compute joint limit cost / smoothness cost / collision cost.
    std::vector<std::future<std::tuple<double, Eigen::VectorXd, Eigen::VectorXd>>> costs;
    std::vector<double> cost_times;
    for (int i = 0; i < discretizations_; i++)
    {
      auto& configuration = configurations_[i];
      const auto t = static_cast<double>(i) / (discretizations_ - 1) * trajectory_num_curves_;
      cost_times.push_back(t);

      Eigen::VectorXd robot_positions(joint_names.size());
      Eigen::VectorXd robot_velocities(joint_names.size());
      for (int j = 0; j < joint_names.size(); j++)
      {
        robot_positions(j) = trajectory_->getSpline(joint_names[j]).position(t);
        robot_velocities(j) = trajectory_->getSpline(joint_names[j]).velocity(t);
      }

      configuration.setPositions(robot_positions);
      configuration.setVelocities(robot_velocities);

      configuration.forwardKinematics();
      configuration.computeGripperTransformDerivative();

      // Compute the cost and gradient conditionally asynchronously
      costs.push_back(std::async([optimizer = this, &configuration]()
                                 { return optimizer->computeCost(configuration); }));
    }

    // Compute objective completion cost
    for (int i = 0; i < objectives_.size(); i++)
    {
      auto& configuration = configurations_[discretizations_ + i];
      const auto t = objective_completion_times_[i] / trajectory_duration_ * trajectory_num_curves_;
      cost_times.push_back(t);

      Eigen::VectorXd robot_positions(joint_names.size());
      Eigen::VectorXd robot_velocities(joint_names.size());
      for (int j = 0; j < joint_names.size(); j++)
      {
        robot_positions(j) = trajectory_->getSpline(joint_names[j]).position(t);
        robot_velocities(j) = trajectory_->getSpline(joint_names[j]).velocity(t);
      }

      configuration.setPositions(robot_positions);
      configuration.setVelocities(robot_velocities);

      configuration.forwardKinematics();
      configuration.computeGripperTransformDerivative();

      // Compute the cost and gradient conditionally asynchronously
      costs.push_back(std::async([optimizer = this, &configuration, i = i]()
                                 { return optimizer->computeObjectiveCost(configuration, i); }));
    }

    //printf("[%lf ms] Created all tasks\n", stopwatch.time() * 1000.);

    // Aggregate gradients
    Eigen::MatrixXd total_position_gradient(joint_names.size(), trajectory_num_curves_);
    Eigen::MatrixXd total_velocity_gradient(joint_names.size(), trajectory_num_curves_);
    total_position_gradient.setZero();
    total_velocity_gradient.setZero();
    for (int i = 0; i < costs.size(); i++)
    {
      const auto t = cost_times[i];

      // Update gradient
      auto cost_tuple = costs[i].get();
      const auto& cost = std::get<0>(cost_tuple);
      const auto& position_gradient = std::get<1>(cost_tuple);
      const auto& velocity_gradient = std::get<2>(cost_tuple);

      // Print cost and gradient
      /*
      std::cout << "[" << i << "] time = " << t << ", cost = " << cost << ", gradient = " << position_gradient.transpose() << ", "
                << velocity_gradient.transpose() << "\n";
                */

      auto curve_index = trajectory_->getSpline(0).getCurveIndex(t);
      auto position_coeff = static_cast<Eigen::Vector4d>(trajectory_->getSpline(0).getPositionCoefficients(t));
      auto velocity_coeff = static_cast<Eigen::Vector4d>(trajectory_->getSpline(0).getVelocityCoefficients(t));

      for (int j=0; j<joint_names.size(); j++)
      {
        if (curve_index == 0)
        {
          total_position_gradient(j, curve_index) += position_gradient(j) * position_coeff(2) / 4.;
          total_velocity_gradient(j, curve_index) += position_gradient(j) * position_coeff(3) / 4.;

          total_position_gradient(j, curve_index) += velocity_gradient(j) * velocity_coeff(2) / 4.;
          total_velocity_gradient(j, curve_index) += velocity_gradient(j) * velocity_coeff(3) / 4.;
        }
        else
        {
          total_position_gradient(j, curve_index-1) += position_gradient(j) * position_coeff(0) / 8.;
          total_velocity_gradient(j, curve_index-1) += position_gradient(j) * position_coeff(1) / 8.;
          total_position_gradient(j, curve_index) += position_gradient(j) * position_coeff(2) / 8.;
          total_velocity_gradient(j, curve_index) += position_gradient(j) * position_coeff(3) / 8.;

          total_position_gradient(j, curve_index-1) += velocity_gradient(j) * velocity_coeff(0) / 8.;
          total_velocity_gradient(j, curve_index-1) += velocity_gradient(j) * velocity_coeff(1) / 8.;
          total_position_gradient(j, curve_index) += velocity_gradient(j) * velocity_coeff(2) / 8.;
          total_velocity_gradient(j, curve_index) += velocity_gradient(j) * velocity_coeff(3) / 8.;
        }
      }
    }

    //std::cout << "gradients = \n" << total_position_gradient << "\n" << total_velocity_gradient << "\n";

    //printf("[%lf ms] Joined all tasks\n", stopwatch.time() * 1000.);

    // Move the trajectory along the gradient.
    const double alpha = 1e-1;
    for (int i=0; i<joint_names.size(); i++)
    {
      for (int j=1; j<=trajectory_num_curves_; j++)
      {
        trajectory_->getSpline(i).controlPosition(j) -= alpha * total_position_gradient(i, j-1);
        trajectory_->getSpline(i).controlVelocity(j) -= alpha * total_velocity_gradient(i, j-1);
      }
    }

    // Print trajectory control points
    /*
    printf("trajectory = \n");
    for (int i=0; i<joint_names.size(); i++)
    {
      for (int j=0; j<=trajectory_num_curves_; j++)
        printf("%lf ", trajectory_->getSpline(i).controlPosition(j));
      printf("    ");
      for (int j=0; j<=trajectory_num_curves_; j++)
        printf("%lf ", trajectory_->getSpline(i).controlVelocity(j));
      printf("\n");
    }
     */


    // TODO: optimize over the objective completion time
    // Need to compute objective completion cost.
    // Forward (velocity) kinematics at the varying objective completion times.

    // Compute the gradient. Each component has +w because it is added to the overall cost minimizing the sum of task
    // completion time.

    // Move the objective completion time along the gradient.


    /*
    printf("Objective completion times:");
    for (int i = 0; i < objective_completion_times_.size(); i++)
      printf(" %lf", objective_completion_times_[i]);
    printf("\n");
     */
  }
}

std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> Planner::computeCost(const RobotConfiguration& configuration)
{
  double cost = 0.;
  Eigen::VectorXd position_gradient(configuration.getPositions().rows());
  Eigen::VectorXd velocity_gradient(configuration.getVelocities().rows());
  position_gradient.setZero();
  velocity_gradient.setZero();

  for (int i = 0; i < costs_.size(); i++)
  {
    auto result = costs_[i]->computeCost(configuration);

    cost += std::get<0>(result);
    position_gradient += std::get<1>(result);
    velocity_gradient += std::get<2>(result);
  }

  return std::make_tuple(cost, position_gradient, velocity_gradient);
};

std::tuple<double,
           Eigen::VectorXd,
           Eigen::VectorXd> Planner::computeObjectiveCost(const RobotConfiguration& configuration, int objective_index)
{
  auto result = objectives_[objective_index]->computeCost(configuration);

  double cost = std::get<0>(result);
  auto position_gradient = std::get<1>(result);
  auto velocity_gradient = std::get<2>(result);

  return std::make_tuple(cost, position_gradient, velocity_gradient);
};

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

    // TODO: trajectory_duration_ is never used in trajectory. Replace it
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
  const auto num_links = robot_model_->numLinks();

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

      frame->transform = object->getTransform();
    }
    else if (auto cylinder = dynamic_cast<Cylinder*>(shape.get()))
    {
      custom_mesh->createCylinder(cylinder->getRadius(), cylinder->getHeight());
      custom_mesh->setGlobalColor(Eigen::Vector3f(color(0), color(1), color(2)));

      frame->transform = object->getTransform();
    }
    else if (auto sphere = dynamic_cast<Sphere*>(shape.get()))
    {
      custom_mesh->createSphere(sphere->getRadius());
      custom_mesh->setGlobalColor(Eigen::Vector3f(color(0), color(1), color(2)));

      frame->transform.translate(object->getTransform().translation());
    }

    renderer_publisher_.publish(std::move(frame));
    renderer_publisher_.publish(std::move(custom_mesh));
  }
}

void Planner::drawRobotStatus(const Eigen::VectorXd& p, std::string tag)
{
  RobotConfiguration configuration(planning_robot_model_);
  configuration.setPositions(p);
  configuration.forwardKinematics();
  configuration.computeGripperTransformDerivative();

  int cube_id = 0;

  // TODO: refactoring
  static std::unordered_set<std::string> tag_check;
  bool attach_mesh = false;
  if (tag_check.find(tag) == tag_check.end())
  {
    attach_mesh = true;
    tag_check.insert(tag);
  }

  for (int i = 0; i < planning_robot_model_->numLinks(); i++)
  {
    const auto& link = planning_robot_model_->getLink(i);
    const auto& bounding_volumes = link.getBoundingVolumes();

    auto frame = std::make_unique<RequestFrame>();
    frame->action = RequestFrame::Action::Set;
    frame->name = "status_" + tag + "_" + std::to_string(i);
    frame->transform = configuration.getTransform(i);
    renderer_publisher_.publish(std::move(frame));

    // Bounding volumes
    for (const auto& bounding_volume : bounding_volumes)
    {
      const auto& cube = bounding_volume->as<Cube>();

      auto frame = std::make_unique<RequestFrame>();
      frame->action = RequestFrame::Action::Set;
      frame->name = "bv_" + tag + "_" + std::to_string(cube_id);
      frame->parent = "status_" + tag + "_" + std::to_string(i);
      frame->transform = cube.getTransform();
      renderer_publisher_.publish(std::move(frame));

      if (attach_mesh)
      {
        auto custom_mesh = std::make_unique<RequestCustomMesh>();
        custom_mesh->name = "bv_" + tag + "_" + std::to_string(cube_id);
        custom_mesh->frame = custom_mesh->name;
        custom_mesh->createCube(cube.getSize());
        renderer_publisher_.publish(std::move(custom_mesh));
      }

      cube_id++;
    }

    // TODO: attach visual meshes
  }

  // Gripper
  auto frame = std::make_unique<RequestFrame>();
  frame->action = RequestFrame::Action::Set;
  frame->name = "gripper";
  frame->transform = configuration.getGripperTransformFromBase();
  renderer_publisher_.publish(std::move(frame));

  if (attach_mesh)
  {
    auto custom_mesh = std::make_unique<RequestCustomMesh>();
    custom_mesh->name = "gripper_marker";
    custom_mesh->frame = "gripper";
    custom_mesh->createCube(Eigen::Vector3d(0.1, 0.1, 0.1));
    custom_mesh->setGlobalColor(Eigen::Vector3f(1.f, 0.f, 0.f));
    renderer_publisher_.publish(std::move(custom_mesh));
  }

  // Todo: Gripper derivative
  for (int i = 0; i < planning_robot_model_->numJoints(); i++)
  {
  }
}

void Planner::drawTrajectory()
{
  const auto n = planning_robot_model_->numJoints();

  for (int i = 0; i < discretizations_; i++)
  {
    auto& configuration = configurations_[i];
    const auto t = static_cast<double>(i) / (discretizations_ - 1) * trajectory_num_curves_;

    Eigen::VectorXd robot_positions(n);

    for (int j = 0; j < n; j++)
      robot_positions(j) = trajectory_->getSpline(j).position(t);

    drawRobotStatus(robot_positions, std::to_string(i));
  }
}
}
