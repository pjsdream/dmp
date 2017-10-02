#ifndef DMP_COST_H
#define DMP_COST_H

#include <dmp/comm/message.h>

#include <Eigen/Dense>

namespace dmp
{
class RobotConfiguration;

class Cost : public Message
{
public:
  Cost() = delete;
  explicit Cost(double weight);
  virtual ~Cost() = default;

  double getWeight() const noexcept;

  virtual std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> computeCost(const RobotConfiguration& configuration);

private:
  double weight_;
};
}

#endif //DMP_COST_H
