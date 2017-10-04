#include <dmp/controlling/controller.h>
#include <dmp/controlling/controller_option.h>
#include <dmp/robot/robot_model.h>
#include <dmp/robot/robot_link.h>
#include <dmp/robot/robot_joint.h>
#include <dmp/rendering/request/request_frame.h>
#include <dmp/rendering/request/request_mesh.h>
#include <dmp/utils/rate.h>

namespace dmp
{
Controller::Controller(const ControllerOption& option)
    : Node("controller")
{
  robot_model_ = option.getRobotModel();
}

Subscriber<Trajectory>& Controller::getTrajectorySubscriber()
{
  return trajectory_subscriber_;
}

Publisher<Request>& Controller::getRendererPublisher()
{
  return renderer_publisher_;
}

Publisher<RobotState>& Controller::getRobotStatePublisher()
{
  return robot_state_publisher_;
}

void Controller::drawRobot(const Eigen::VectorXd& joint_values, std::string tag)
{
  auto num_links = robot_model_->numLinks();

  // TODO: remove static local variable (using int temporarily)
  static bool mesh_attached = false;

  for (int i = 0; i < num_links; i++)
  {
    const auto& link = robot_model_->getLink(i);

    // frame
    // TODO: send requests at once. Currently, it's sending one by one.
    const std::string frame_name{std::string("model_") + tag + "_" + link.getName()};
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

      frame->parent = std::string("model_") + tag + "_" + parent_link.getName();

      // joint id 0 is a dummy, so use i-1 for joint index (because i is the link index)
      frame->transform = joint.getJointTransform(joint_values[i - 1]);
    }
    renderer_publisher_.publish(std::move(frame));

    // TODO: remove static local variable (using int temporarily)
    if (!mesh_attached)
    {
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
    }
  }

  // TODO: remove static local variable (using int temporarily)
  //if (!mesh_attached)
  //  mesh_attached = true;
}

void Controller::run()
{
  using namespace std::chrono_literals;

  Rate rate(1);

  for (int i = 0; i < 100 * 100; i++)
  {
    auto requests = trajectory_subscriber_.popAll();

    // rendering the current robot state
    // TODO: render the robot state of the recentest trajectory
    // Currently, rendering the robot state of the first point of the requested trajectory
    if (!requests.empty())
    {
      const auto& request = requests[0];

      int cnt = 0;
      for (const auto& point : request->getPoints())
      {
        const auto& body_joint_values = point.getJointPositions();

        Eigen::VectorXd joint_values(robot_model_->numJoints());
        const auto& whole_body_joint_names = robot_model_->getJointNames();
        const auto& trajectory_joint_names = request->getJointNames();

        for (int j = 0; j < whole_body_joint_names.size(); j++)
        {
          joint_values[j] = 0.;

          for (int k = 0; k < trajectory_joint_names.size(); k++)
          {
            if (whole_body_joint_names[j] == trajectory_joint_names[k])
            {
              joint_values[j] = body_joint_values[k];
              break;
            }
          }
        }

        drawRobot(joint_values, std::to_string(cnt));
        cnt++;
      }
    }

    rate.sleep();
  }
}
}
