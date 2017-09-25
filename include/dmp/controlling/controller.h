#ifndef DMP_CONTROLLER_H
#define DMP_CONTROLLER_H

#include <dmp/comm/node.h>
#include <dmp/comm/publisher.h>
#include <dmp/comm/subscriber.h>
#include <dmp/rendering/request/request.h>
#include <dmp/trajectory/trajectory.h>

namespace dmp
{
class ControllerOption;
class RobotModel;

class Controller : public Node
{
public:
  Controller() = delete;
  explicit Controller(const ControllerOption& option);

  Subscriber<Trajectory>& getTrajectorySubscriber();
  Publisher<Request>& getRendererPublisher();

protected:
  void run() override;

private:
  void drawRobot(const Eigen::VectorXd& joint_values);

  Subscriber<Trajectory> trajectory_subscriber_;
  Publisher<Request> renderer_publisher_;

  std::shared_ptr<RobotModel> robot_model_;
};
}

#endif //DMP_CONTROLLER_H
